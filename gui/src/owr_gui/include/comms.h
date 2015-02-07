#ifndef H_COMMS
#define H_COMMS

#define UPDATE_CONST_FUNCTION_DEF (float, float,float, ListNode, vector2D)

typedef struct _vector2D * ListNode;

typedef struct _vector2D {
    double x;
    double y;
    ListNode next;
} vector2D;

void updateConstants(float battery, float signal, float ultrasonic, ListNode points, vector2D target, unsigned char *frame);

#endif
