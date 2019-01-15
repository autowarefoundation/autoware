Header header

uint32 width
uint32 height

# each uint8 or uint16 contains position of white pixel expressed like: "(x << 8 or 16) ^ y"
# if max(width, height) > 256(=2^8) use data32 array, else use data16 array.
uint16[] data16
uint32[] data32
