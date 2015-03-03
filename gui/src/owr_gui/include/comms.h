#ifndef H_COMMS
#define H_COMMS

#define UPDATE_CONST_FUNCTION_DEF (float, float,float, ListNode, vector2D)

typedef struct {
	double x;
	double y;
} vector2D, *ListNode;

void updateConstants(float bat, float sig, float ultrason, ListNode cur, vector2D target, unsigned char *frame);

#endif
