#include "stdio.h"

struct aaa{
    int a;
    int b;
};

int  main(void) {
    aaa test[4] = {{1, 2}, {3, 4}, {5, 6}, {7, 8}};
    struct aaa* ptr = &test[0];

    int c = (test+0)->a;
    printf("Hello, %d\n",c);
    
     c = (test+1)->a;
    printf("Hello, %d\n",c);
    
     c = (test+2)->a;
    printf("Hello, %d\n",c);
    
    printf("Hello, BridgeCore Test!\n");
    return 0;
}