
#define BIT_IS_SURFACE   0  
#define BIT_MERGE_TYPE   1  // 1-3 (requires 3 bits)

#define BIT_EXPAND_X     2  // only used when BIT_IS_SURFACE is set to 1
#define BIT_EXPAND_Y     3  // only used when BIT_IS_SURFACE is set to 1


// merge types
#define MERGE_NONE (0 << BIT_MERGE_TYPE)
#define MERGE_ALL  (1 << BIT_MERGE_TYPE)
#define MERGE_TB   (2 << BIT_MERGE_TYPE)
#define MERGE_T    (3 << BIT_MERGE_TYPE)
#define MERGE_B    (4 << BIT_MERGE_TYPE)
#define MERGE_LR   (5 << BIT_MERGE_TYPE)
#define MERGE_L    (6 << BIT_MERGE_TYPE)
#define MERGE_R    (7 << BIT_MERGE_TYPE)


#define BIT_CONTINUOUS_T    4
#define BIT_CONTINUOUS_R    5
#define BIT_CONTINUOUS_B    6
#define BIT_CONTINUOUS_L    7

#define BIT_CONTINUOUS_TR   8
#define BIT_CONTINUOUS_TL   9
#define BIT_CONTINUOUS_BR   10
#define BIT_CONTINUOUS_BL   11


#define BIT_CURRENT_LEVEL   12 // 12-15 (requires 3 bits)

#define ALL_MERGE_TYPE_BITS 14
#define ALL_CONTINUITY_BITS 4080
#define ALL_DATA_BITS       4095


