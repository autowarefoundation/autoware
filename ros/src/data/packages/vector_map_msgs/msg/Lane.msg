# jct
uint8 NORMAL=0
uint8 LEFT_BRANCHING=1
uint8 RIGHT_BRANCHING=2
uint8 LEFT_MERGING=3
uint8 RIGHT_MERGING=4
uint8 COMPOSITION=5

# lanetype
uint8 STRAIGHT=0
uint8 LEFT_TURN=1
uint8 RIGHT_TURN=2

# lanecfgfg
uint8 PASS=0
uint8 FAIL=1

# Ver 1.00
int32 lnid
int32 did
int32 blid
int32 flid
int32 bnid
int32 fnid
int32 jct
int32 blid2
int32 blid3
int32 blid4
int32 flid2
int32 flid3
int32 flid4
int32 clossid
float64 span
int32 lcnt
int32 lno

# Ver 1.20
int32 lanetype
int32 limitvel
int32 refvel
int32 roadsecid
int32 lanecfgfg
int32 linkwaid
