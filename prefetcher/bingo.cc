#include <boolean>
#include <vector>
#include <functional>

#include "cache.h"
#include "champsim.h"

// Idea 是否考虑进程呢？
// 能否引入criticality-based scheduling

// ref: https://github.com/0xd3ba/build-your-own-prefetcher/tree/main/3_write_a_prefetcher_p2
/* Convenient macros to extract out page ID and block ID from a given 64-bit address */
#define EXTRACT_PAGE_ID(addr)   ((addr) >> LOG2_PAGE_SIZE)              /* Extract the page ID */
#define EXTRACT_BLOCK_ID(addr)  (((addr) >> LOG2_BLOCK_SIZE) & 0x3f)    /* Extract the block ID within the page */

/* Utility method to prepare the address to prefetch */
uint64_t prepare_prefetch_address(uint64_t page_id, uint32_t block_id) {
    return (page_id << LOG2_PAGE_SIZE) + (block_id << LOG2_BLOCK_SIZE);
}

/* Minimum and maximum value of the block IDs */
#define BLOCK_ID_MIN    0
#define BLOCK_ID_MAX    ((PAGE_SIZE / BLOCK_SIZE) - 1)


#define NUM_CPU 4
// See Spatial Memory Stream section 4.5
// number of entry in accumulation table
#define NUM_ACC_ENTRY 32
// number of entry in filter table
#define NUM_FIL_ENTRY 64

// TODO
// the number of way of history table
#define NUM_HIS_WAY 16
// the number of set of history table
#define NUM_HIS_SET 16 * 1024

typedef bool pattern[BLOCK_IN_MAX+1];

/*
  Spatial patterns are recorded in the accumulation table.
  Entries in the accumulation table are tagged by the spatial region tag,
  the high order bits of the region base address.
  Each entry stores the PC and spatial region offset of the trigger access,
  and a spatial pattern bit vector indicating which blocks have been accessed during the generation.
 */
// the entry of accumulation table.
class acc_entry {
private:
  bool valid;
  // the pc of trigger access
  // in order to be consistent with the interface of the original API, hence, we use ip, rather pc.
  uint64_t ip;
  // tag: region base address(page_id)
  uint64_t tag;
  // the data address of trigger access
  // In the original paper Spatial Memory Streaming, the field of entry of accumulation table is Tag, PC/Offset, Pattern
  // But we change PC/Offset field to PC/Address to avoid to computer address of trigger access when transfer entry from accumulation
  // table to pattern history table.
  uint64_t addr;
  int32_t recency;
  // pattern
  pattern& footprint;
public:  
  acc_entry(bool valid, int32_t recency, pattern & footprint): valid(valid),
							       recency(recency),
							       footprint(footprint) {}
  
  bool get_valid() const {
    return valid;
  }

  uint64_t get_ip() const {
    return ip;
  }
  
  uint64_t get_tag() const {
    return tag;
  }
  
  uint64_t get_addr() const {
    return addr;
  }
  
  int32_t get_recency() const {
    return recency;
  }

  pattern& get_footprint() const {
    return footprint;
  }
  
  void set_valid(bool valid) {
    valid = valid;
  }

  void set_ip(uint64_t ip) {
    ip = ip;
  }
  
  void set_tag(uint64_t tag) {
    tag = tag;
  }

  void set_addr(uint64_t addr) {
    addr = addr;
  }
  
  void set_recency(int32_t recency) {
    recency = recency;
  }

  void update_footprint(uint32_t offset) {
    pattern[offset] = true;
  }
};

// accumulation table
class accumulation {
  // Entries in the accumulation table are tagged by the spatial region tag
  // tag: region base address(page_id), trigger access pc/offset, Pattern
private:
  acc_entry& table[NUM_ACC_ENTRY];
public:
  accumulation() {
    for (int i = 0; i < NUM_ACC_ENTRY; i++) {
      pattern& footprint(BLOCK_ID_MAX+1) = {false};
      acc_entry& entry(false, i, footprint);
      table[i] = entry;
    }
  }

  bool get_valid(int32_t way) {
    return table[way]->get_valid();
  }
  uint64_t get_ip(int32_t way) {
    return table[way]->get_ip();
  }

  uint64_t get_addr(int32_t way) {
    return table[way]->get_addr();
  }

  pattern & get_footprint(int32_t way) {
    return table[way]->get_footprint();
  }
  
  // Searches the table for an element which tag equal to tag 
  // tag: page_id(or region base address) in the original paper
  int32_t find(uint64_t tag) {
    for (int i = 0; i < NUM_ACC_ENTRY; i++) {
      if (table[i]->get_tag() == tag && table[i]->get_valid()) {
	return i;
      }
    }
    return -1;
  }
  
  // set the spatial pattern bit corresponding to the accessed block if found,
  void update(uint64_t tag, uint32_t offset) {
    for (acc_entry& entry : table) {
      if (entry->get_tag() == tag) {
	entry->update_pattern(offset);
      }
    }
  }

  uint32_t lru_victim() {
    uint32_t way = 0;

    // fill invalid line first
    for (way = 0; way < NUM_ACC_ENTRY; way++) {
      if(!table[way]->get_valid()) {
	break;
      }
    }
    
    // LRU victim
    if (way == NUM_ACC_ENTRY) {
      for (way=0; way < NUM_WAY; way++) {
	if (table[way]->get_recency() == NUM_ACC_ENTRY-1) {
	  break;
	}
      }
    }

    // LRU update
    for (uint32_t i = 0; i < NUM_WAY; i++) {
      if (table[i]->get_recency() < table[i]->get_recency()) {
	table[i]->set_recency(table[i]->get_recency()+1);
      }
    }
    return way;
  }

  // ip, tag, offset are field of entry evicted from of filter table
  // addr: data address of new access
  uint32_t allocate(uint64_t tag, uint64_t ip, uint64_t trigger_addr, uint64_t addr, uint32_t way) {
    table[way]->set_recency(0);
    table[way]->set_tag(tag);
    table[way]->set_ip(ip);
    table[way]->set_valid(true);
    table[way]->set_addr(trigger_addr);
    // trigger accessed block
    table[way]->update_pattern(EXTRACT_BLOCK_ID(trigger_addr));
    // this accessed block
    table[way]->updata_pattern(EXTRACT_BLOCK_ID(addr));
  }
  
  // eviction ends generation and sends pattern to pattern history table
   void free(uint64_t tag) {
    int32_t way = find(tag);
    if (way == -1) return;
    table[way]->set_valid(false);
  }
}


/*
  New spatial region generations are initially allocated in the filter table.
  The filter table records the spatial region tag, and the PC and spatial region
  offset of the trigger access, for spatial regions that have had only a single access in their current generation. 
 */
class fil_entry {
private:
  bool valid;
  uint64_t tag;
  uint64_t ip;
  uint64_t addr;
  uint32_t recency;
public:
  fil_entry(bool valid, uint64_t addr): valid(valid),
					recency(recency) {};
  bool get_valid() const {
    return valid;
  }
  
  uint64_t get_tag() const {
    return tag;
  }
  
  uint64_t get_ip() const {
    return pc;
  }
  
  uint64_t get_addr() const {
    return addr;
  }
  
  uint32_t get_recency() const {
    return recency;
  }
  
  void set_valid(bool valid) {
    valid = valid;
  }
  
  void set_tag(uint64_t tag) {
    tag = tag;
  }
  
  void set_ip(uint64_t ip) {
    ip = ip;
  }
  
  void set_addr(uint64_t addr) {
    addr = addr;
  }
  
  void set_recency(uint32_t recency) {
    recency = recency;
  }
}

// filter table
class filter {
private:
  fil_entry & table[NUM_FIL_ENTRY];
public:
  fil_table() {
    for (int i = 0; i < NUM_FIL_ENTRY; i++) {
      fil_entry& entry(false, i);
      table[entry];
    }
  }
  
  int32_t find(uint64_t tag) {
    for (int i = 0; i < NUM_FIL_ENTRY; i++) {
      if (table[i]->get_tag() ==tag && table[i]->get_valid()) {
	return i;
      }
    }
    return -1;
  }

  uint64_t get_tag(int32_t way) const {
    return table[way]->get_tag();
  }
  uint64_t get_ip(int32_t way) const {
    return table[way]->get_ip();
  }
  uint64_t get_addr(int32_t way) const {
    return table[way]->get_addr();
  }
  uint32_t lru_victim() {
    uint32_t way = 0;
    
    // fill invalid line first
    for (way = 0; way < NUM_ACC_ENTRY; way++) {
      if(!table[way]->get_valid()) {
	break;
      }
    }
    
    // LRU victim
    if (way == NUM_ACC_ENTRY) {
      for (way=0; way < NUM_WAY; way++) {
	if (table[way]->get_recency() == NUM_FIL_ENTRY-1) {
	  break;
	}
      }
    }
    
    // LRU update
    for (uint32_t i = 0; i < NUM_WAY; i++) {
      if (table[i]->get_recency() < table[i]->get_recency()) {
	table[i]->set_recency(table[i]->get_recency()+1);
      }
    }
    return way;
  }

  // allocate an new entry in filter table when a spatial region generation begins
  // ip, tag, offset are field of entry evicted from of filter table
  // addr: data address of new access
  uint32_t allocate(uint64_t tag, uint64_t ip, uint64_t addr, uint32_t way) {
    table[way]->set_recency(0);
    table[way]->set_tag(tag);
    table[way]->set_ip(ip);
    table[way]->set_valid(true);
    table[way]->set_addr(addr);
  }
  
  // eviction ends generation and sends pattern to pattern history table
  void free(uint64_t tag) {
    int32_t way = find(tag);
    if (way == -1) return;
    table[way]->set_valid(false);
  }
}

/* active generation table records spatial patterns as the processor
   accesses spatial regions and trains the predictor.
 */
class AGT {
private:
  filter fil_table;
  accumulation acc_table;
public:
  // initialize fil_table and acc_table
  AGT() {
    fil_table = filter();
    acc_table = accumulation();
  }

  // if tag existing in acc_table, return true
  int32_t find(uint64_t tag) {
    return acc_table.find(tag);
  }

  // When call these three get method, you should first call find(uint64_t tag)
  // and make sure the return value is true.
  // tag: page_id
  uint64_t get_ip(int32_t way) {
    return acc_table.get_ip(way);
  }
  
  uint64_t get_addr(int32_t way) {
    return acc_table.get_addr(way);
  }

  pattern& get_footprint(int32_t way) {
    return acc_table.get_footprint(way);
  } 

  bool is_trigger(uint64_t tag) {
    return acc_table.find(tag) != -1 || fil_table.find(tag) != -1;
  }

  // return the way of valid victimed entry in accumulation table
  // otherwise, return -1
  int32_t lru_victim() {
    int32_t way = acc_table.alu_victim();
    if (acc_table[way]->get_valid()) {
      return way;
    }
    return -1;
  }
  /*
    When a spatial region generation begins, SMS allocates an entry in the AGT.
    As cache blocks are accessed, SMS updates the recorded pattern in the AGT.
   */
  // store a new entry, return the tag of evicted entry if the accumulation table is full
  // otherwise, return 0;
  uint64_t record(uint64_t ip, uint64_t addr) {
    uint64_t tag = EXTRACT_PAGE_ID(addr);
    uint32_t offset = EXSTRACT_PAGE_ID(addr);
    
    if (find(tag) != -1) {
      //  Each L1 access first searches the accumulation table. If a matching entry is found,
      // the spatial pattern bit corresponding to the accessed block is set.
      acc_table->update(tag, offset);
      return 0;
    }
    // Otherwise, the access searches for its tag in the filter table. If no match is found,
    // this access is the trigger access for a new spatial region generation and a new entry
    // is allocated in the filter table.
    uint32_t index = fil_table.find(tag);
    if (index == -1) {
      uint32_t way = fil_table.lru_victim();
      fil_table.allocate(ip, tag, addr, way);
      return 0;
    }
    // If an access matches in the filter table, its spatial region offset is compared to the recorded offset.
    // If the offsets differ, then this block is the second distinct cache block accessed within the generation,
    // and the entry in the filter table is transferred to the accumulation table (step 2). Additional
    if (fil_table.get_addr(index) != addr) {
      uint32_t way = acc_table.lru_victim();
      uint32_t ret_way = 0;
      if (acc_table.get_valid(way)) {
	ret_way = way;
      }
      acc_table.allocate(tag, fil_table.get_ip(index), fil_table.get_addr(index), addr, way);
      fil_table.free(index);
      return way;
    }
  }

  /*
    At the end of a generation (eviction/invalidation of any block accessed during the generation),
    the AGT transfers the spatial pattern to the history table and the AGT entry is freed.
  */
  // return true if the eviction of block which address is victim_addr cause the end of one spatial region generation
  // The spatial region might just accessed for one time, so do not forget to check the filter table.
  void free(uint64_t tag) {
    acc_table.free(tag);
    fil_table.free(tag);
  }
}

/*
  pattern history table of SMS
 */

/*
 pattern history table of Bingo

 The table should only be indexed with a hash of the shortest event ("PC + Offset") but
 tagged with the longest event ("PC + Address").

 The bits corresponding to the shortest event are used for indexing the history table to
 find the set in which the metadata should be stored; however, all bits of the longest event
 is used to tag the entry.

 Whenever a new footprint is going to be stored in the metadata organization, it is associated
 with the corresponding "PC + Address". To find a location in the history table for the new entry,
 a hash of only "PC + Offset" is used to index the table. By knowing the set, the baseline replacement
 algorithm(e.g., LRU) is used to choose a victim to open a room for storing the new entry. After determining
 the location, the entry is stored in the history table, but all bits of the 'PC + Address' are used for
 tagging the entry.


 Question:
 what hash function to be used?
 How much set in the table? and How much way in each set?
 */

// the entry of history table, tagged by PC + Address
class his_entry {
private:
  bool valid;
  // tag: ip + addr
  // ip: program count
  uint64_t ip;
  // addr: data address
  uint64_t addr;

  int32_t recency;
  // access pattern: bit vector
  pattern& footprint = {false};
  
public:
  his_entry(int32_t recency, bool valid): recency(recency),
					  valid(valid){}
  
  bool get_valid() cosnt {
    return valid;
  }

  uint64_t get_ip() const {
    return ip;
  }

  uint64_t get_addr() const {
    return addr;
  }

  int32_t get_recency() const {
    return recency;
  }

  pattern& get_footprint() const {
    return footprint;
  }

  void set_valid(bool valid) {
    valid = valid;
  }

  void set_ip(uint64_t ip) {
    ip = ip;
  }

  void set_addr(uint64_t addr) {
    addr = addr;
  }

  void set_recency(int32_t recency) {
    recency = recency;
  }

  void set_footprint(pattern& footprint) {
    footprint = footprint;
  }
}

// the set of history table
class his_set {
private:
  his_entry & set[NUM_HIS_WAY];
public:
   his_set() {
    for (int i = 0; i < NUM_HIS_WAY; i++) {
      his_entry& entry(i, false);
      set[i] = entry;
    }
  }

  uint32_t lru_victim() {
    uint32_t way = 0;
    // fill invalid line first
    for (way = 0; way < NUM_HIS_WAY; way++) {
      if(!set[way]->get_valid()) {
	break;
      }
    }
    
    // LRU victim
    if (way == NUM_HIS_WAY) {
      for (way=0; way < NUM_HIS_WAY; way++) {
	if (set[way]->get_recency() == NUM_HIS_WAY-1) {
	  break;
	}
      }
    }
    
    // LRU update
    for (uint32_t i = 0; i < NUM_HIS_WAY; i++) {
      if (set[i]->get_recency() < set[i]->get_recency()) {
	set[i]->set_recency(set[i]->get_recency()+1);
      }
    }
    return way;
  }

  // allocate an new entry in filter table when a spatial region generation begins
  // ip, tag, offset are field of entry evicted from of filter table
  // addr: data address of new access
  uint32_t replace(uint64_t tag, uint64_t ip, uint64_t addr, pattern& footprint, uint32_t way) {
    set[way]->set_recency(0);
    set[way]->set_ip(ip);
    set[way]->set_valid(true);
    set[way]->set_addr(addr);
    set[way]->set_footprint(footprint);
  }
}

// We use a 16-way set associative structure for the history table
// and set its capacity based on the sensitivity analysis 
// history table
class history {
private:
  his_set & table[NUM_HIS_SET];
public:
  history() {
    for (int i = 0; i < NUM_HIS_SET; i++) {
      his_set & set;
      table[i] = set;
    }
  }

  // get the hash value of ip + offset, then mod NUM_HIS_SET
  uint32_t get_index(uint64_t ip, uint64_t addr) {
    uint32_t offset = EXTRACT_BLOCK_ID(addr);
    // the hash value of pc + offset
    hash<std::pair<uint64_t, uint32_t> > indexhash(make_pair(ip, offset));
    return indexhash % NUM_HIS_SET;
  }
  // store a new entry in the history table
  void store(uint64_t ip, uint64_t addr, pattern& footprint) {
    uint32_t index = get_index(ip, addr);
    int32_t way = table[index].lru_victim();
    table[index].replace(ip, addr, footprint, way);
  }

  std::vector<pattern &> lookup(uint64_t ip, uint64_t addr) {
    uint32_t index = get_index(ip, addr);
    std::vector<pattern &> footprints;
    for (his_entry & entry : table[index]) {
      if (entry->get_valid()) {
	if (entry->get_addr() == addr) {
	  footprints.clear();
	  footprints.push_back(entry->get_footprint());
	  return footprints;
	}
	footprints.push_back(entry->get_footprint());
      }
    }
    return footprints;
  }
}

/* NUM_CPUS is a pre-defined constant set to the number of cores to use in the simulation */
// active generation table
AGT agt[NUM_CPUS];
history hist_table[NUM_CPUS];

/*
  Spatial region generations end with an eviction or invalidation (step 4).
  Upon these events, both the filter table and accumulation table are searched
  for the corresponding spatial region tag. (Note that this search requires
  reading the tags of replaced cache blocks even if the replaced block is clean).
  A matching entry in the filter table is discarded because it represents a
  generation with only a trigger access. A matching entry in the accumulation
  table is transferred to the pattern history table.
 */
void CACHE::end_agt(uint32_t cpu, uint32_t victim_addr) {
  // we choose the interval from the trigger access until any block accessed during
  // the generation is removed from the processor’s primary cache by replacement or invalidation.
  uint64_t tag = EXTRACT_PAGE_ID(victim_addr);
  int32_t way = agt[cpu].find(tag);
  if (way != -1) {
    // TODO
    hist_table[cpu].store(agt[cpu].get_ip(way), agt[cpu].get_addr(way), agt[cpu].get_footprint(way));
  }
  agt[cpu].free(tag);
}

// TODO support multiple CPU
void CACHE::l2c_prefetcher_initialize() 
{
  cout << "CPU " << cpu << " L2C Bingo prefecther" << endl;
}

uint32_t CACHE::l2c_prefetcher_operate(uint64_t addr, uint64_t ip, uint8_t cache_hit, uint8_t type, uint32_t metadata_in)
{
  
  uint64_t tag = EXTRACT_PAGE_ID(addr);        /* Extract out the page ID of the current load/store */

  /* Check if the access is the trigger access */
  /* If no, then just update the active generation table and return */
  /* NOTE: cpu is a member variable of CACHE, which stores the ID of the CPU where the cache belongs */
  if (!agt[cpu].is_trigger(tag)) {
    /*
      If either table is full when a new entry must be allocated, a victim entry
      is selected and the corresponding generation is terminated (i.e., the entry
      is dropped from the filter table or transferred from the accumulation table
      to the pattern history table).
     */

    // A new entry must be allocated in the accumulation table
    if (agt[cpu].find(tag) == -1) {
      int32_t way = agt[cpu].lru_victim();
      if (way != -1) {
	// a valid entry in accumulation must be evicted
	// transferred from the accumulation table to the pattern history
	hist_table.store(agt[cpu].get_ip(way), agt[cpu].get_addr(way), agt[cpu].get_footprint(way));
      }
    }

  /* A trigger access */
  /* A new entry must be allocated in filter table. If the filter table is full, a victim entry just drop */
  agt[cpu].record(ip, addr);                   /* Append the trigger access to the active generation table */

  std::vector<pattern &> footprints = hist_table[cpu].lookup(ip, addr);
  uint64_t prefetch_addr;
  for (auto footprint : footprints) {
    for (int i = 0; i < NUM_BLOCK_ID; i++) {
      if (footprint[i]) {
	prefetch_addr = prepare_prefetch_address(page_id, i);
	prefetch_line(ip, addr, prefetch_addr, FILL_L2, 0);
      }
    }
  }
  return metadata_in;
}

uint32_t CACHE::l2c_prefetcher_cache_fill(uint64_t addr, uint32_t set, uint32_t way, uint8_t prefetch, uint64_t evicted_addr, uint32_t metadata_in)
{
  return metadata_in;
}

void CACHE::l2c_prefetcher_final_stats()
{

}
