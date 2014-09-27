
typedef struct _vector2D {
    double x;
    double y;
    vector2D * next;
} vector2D;

typedef vector2D * ListNode;


void updateConstants(float battery, float signal, ListNode points, vector2D target);
