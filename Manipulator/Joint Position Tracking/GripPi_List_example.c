#include "GripPiList.h"

int main(){
    MyLinkedList *path_head = myLinkedListCreate();
    pathPlanner(0,0,0,100,100,100);
    myLinkedListPrint(path_head);
     
    return(0);
}
