/* whethre using shared memory or not */
#define USE_SHARED_MEM

/* # of cell block calculated by 1 GPU-thread-block */
#define CELL_PER_BLOCK_X 2
#define CELL_PER_BLOCK_Y 2
#define VOTE_CELL_PER_BLOCK_X (CELL_PER_BLOCK_X + 1)
#define VOTE_CELL_PER_BLOCK_Y (CELL_PER_BLOCK_Y + 1)
