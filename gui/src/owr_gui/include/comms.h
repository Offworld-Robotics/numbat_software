#ifndef H_COMMS
#define H_COMMS

typedef struct _vector2D * ListNode;

typedef struct _vector2D {
    double x;
    double y;
    ListNode next;
} vector2D;




void updateConstants(float battery, float signal, ListNode points, vector2D target);

#endif
