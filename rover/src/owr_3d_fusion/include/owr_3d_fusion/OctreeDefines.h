#define NUM_OCT_TREE_CHILDREN 8
#define FIRST_GUESS_DEPTH 6
//this stores the tree nodes

#define CHILD_1 0b00000001
#define CHILD_2 0b00000010
#define CHILD_3 0b00000100
#define CHILD_4 0b00001000
#define CHILD_5 0b00010000
#define CHILD_6 0b00100000
#define CHILD_7 0b01000000
#define CHILD_8 0b10000000

//Important: this number must be prime
#define HASH_MAP_SIZE 100003

//The maximum resolution (min dimensions) that new nodes can be created at
#define MAX_RESOLUTON 0.005 //5mm