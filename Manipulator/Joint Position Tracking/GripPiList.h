#include <stdio.h>
#include <stdlib.h>

typedef struct MyLinkedList{
    int val;
    float x;
    float y; 
    float z;
    int gripper;
    struct MyLinkedList *next;
} MyLinkedList;

// --- Singly Linked list 
MyLinkedList* myLinkedListCreate(void) {
    MyLinkedList *head = (MyLinkedList*)malloc(sizeof(MyLinkedList)); 
    head->val = 1001;
    head->x = 0;
    head->y = 0;
    head->z = 0;
    head->gripper = 130;
    head->next = NULL;
    return head;
}

/*
void* myLinkedListGet(MyLinkedList* obj, int index) {
    // Returns the adress to the indexed node.
    // No list
    if(obj == NULL) return(NULL);
    // List exists, no nodes yet 
    if(obj->next == NULL && obj->val == 1001) return(NULL);
    
    int count = 0;
    while(obj != NULL){
        if(count == index) return(obj);
        obj = obj->next;
        count++;
    }
    
    return(NULL);
}
*/
// --- I don't know where or how the adress of the head node is stored
// --- so, I'm just going to swap it.
void myLinkedListAddAtHead(MyLinkedList* obj, int x_val, int y_val, int z_val, int gripper_val) {
    if(obj == NULL) return;
    
    if(obj->val == 1001){
        // There is no head
        obj->val = 0;
        obj->x = x_val;
        obj->y = y_val;
        obj->z = z_val;
        obj->gripper = gripper_val;
        return;
    } 
    
    MyLinkedList *newNode;// = (MyLinkedList*)malloc(sizeof(MyLinkedList)); 

    if(obj->next != NULL){
        newNode->next = obj->next;
    }
    else{
        newNode->next = NULL;     
    }
    
    newNode->x = x_val;
    newNode->y = y_val;
    newNode->z = z_val;
    newNode->gripper = gripper_val;
    newNode->next = obj;
    return;

}

void myLinkedListAddAtTail(MyLinkedList* obj, int x_val, int y_val, int z_val, int gripper_val) {
    // There is no list
    if(obj == NULL) return;
    
    // There is no head
    if(obj->val == 1001){
        obj->val = 0;
        obj->x = x_val;
        obj->y = y_val;
        obj->z = z_val;
        obj->gripper = gripper_val;
        return;
    }
    
    MyLinkedList *newNode = (MyLinkedList*)malloc(sizeof(MyLinkedList));
    newNode->x = x_val;
    newNode->y = y_val;
    newNode->z = z_val;
    newNode->gripper = gripper_val;
    newNode->next = NULL;
    
    MyLinkedList *last_node = (MyLinkedList*)malloc(sizeof(MyLinkedList)); 

    //void *last_node = NULL;

    while(obj != NULL){
        last_node = obj;
        obj = obj->next;
    }
    
    obj = last_node;
    obj->next = newNode;
    free(last_node);
}

void myLinkedListAddAtIndex(MyLinkedList* obj, int index, int x_val, int y_val, int z_val, int gripper_val) {
    // There is no list
    if(obj == NULL) return;
    
    // Add at the head
    if(index == 0){
        myLinkedListAddAtHead(obj, x_val, y_val, z_val, gripper_val);
        return;
    }

    if(index == 1 && obj->val == 1001) return;
    MyLinkedList *newNode = (MyLinkedList*)malloc(sizeof(MyLinkedList));
    newNode->x = x_val;
    newNode->y = y_val;
    newNode->z = z_val;
    newNode->gripper = gripper_val;

    MyLinkedList *last_node = (MyLinkedList*)malloc(sizeof(MyLinkedList)); 
    //void *last_node = NULL;
    int count = 0;
    
    while(obj != NULL){
        count++;
        last_node = obj;
        obj = obj->next;
        
        if(count == index){
            //swap nodes
            newNode->next = obj;
            obj = last_node;
            obj->next = newNode;
            free(last_node);
            return;
        }
    }
}

void myLinkedListDeleteAtIndex(MyLinkedList* obj, int index) {
    // There is no list
    if(obj == NULL) return;
    MyLinkedList *last_node = (MyLinkedList*)malloc(sizeof(MyLinkedList)); 
    MyLinkedList *kill_node = (MyLinkedList*)malloc(sizeof(MyLinkedList)); 

    // void *last_node = NULL, *kill_node = NULL;
    int count = 0;
    
    // The head cannot be deleted because leetcode keeps the head address in a struct that is not
    // acessible. This janky solution swaps the values of the next node to the head and then deletes       // it
    if(index == 0){
        if(obj->next != NULL){
            obj->val =  obj->next->val;
            obj->next = obj->next->next;
            kill_node = obj->next;
            // free(kill_node); breaks the program
        }
        else{
            obj->val = 1001;
        }
        free(kill_node);
        free(last_node);
        return;
    }
    
    while(obj != NULL){
        if(count == index){
            //swap nodes
            kill_node = obj;
            obj = last_node;
            obj->next = obj->next->next;
            free(kill_node);
            return;
        }
        count++;
        last_node = obj;
        obj = obj->next;
    }
}

void myLinkedListFree(MyLinkedList* obj) {
    if(obj == NULL) return;

    //void *kill_node = NULL;
    MyLinkedList *kill_node = (MyLinkedList*)malloc(sizeof(MyLinkedList)); 

    while(obj != NULL){
        kill_node = obj;
        obj = obj->next;
        free(kill_node);
    }
}

void myLinkedListPrint(MyLinkedList *obj){
    int index = 0;
    while(obj != NULL){
        printf("Waypoint #%i \n", index);
        printf("%i \n", obj->x);
        printf("%i \n", obj->y);
        printf("%i \n", obj->z);
        printf("%i \n \n", obj->gripper);
        index++;
        obj = obj->next;
    }
}
