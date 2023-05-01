#include "GripPiList.h"

int main(){
    MyLinkedList *path_head = myLinkedListCreate();
    void *head = path_head;
    // myLinkedListAddAtTail(MyLinkedList* obj, int x_val, int y_val, int z_val, int gripper_val)
    myLinkedListAddAtTail(path_head, 999, 998, 997, 996, 90);
    
    //head is the first node in the list. 
    // to return to the first node call path_head = head
    // to along the list path_head = path_head->next;
    // the final node returns a NULL type for path_head->next;
     
    return(0);
}
