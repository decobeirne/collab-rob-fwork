/*!
\file List.h
Double linked list.

The payload in each list node is stored as a void*.
*/

#ifndef LIST_H
#define LIST_H

#include "RobotDefs.h"

typedef struct ListNode_ *ListNodePtr;
typedef struct ListNode_
{
	ListNodePtr prev; //!< Pointer to previous element in list
	ListNodePtr next; //!< Pointer to next element in list
	void *value; //!< Value stored in the list node
} ListNode;

typedef struct List_
{
	int size; //!< Number of elements in list
	ListNode *front; //!< Pointer to first element in list
	ListNode *back; //!< Pointer to last element
} List;

//! Constructor
List initList();

//! Constructor
ListNode initListNode();

// ! Deallocate list node
void ListNode_clear (
	ListNode *ln,
	const int deallocate);

//! Deallocate list node with provided deallocate function for payload.
void ListNode_clearWithDeallocator (
	ListNode *ln,
	void (*deallocate)(void *v));

//! Clear list elements. Free payload of each if specified.
void List_clear (List *l, const int deallocate);

//! Clear list elements using provided deallocate function.
void List_clearWithDeallocator (List *l, void (*deallocator)(void *v));

//! Push value.
void List_pushValue (
	List *l,
	void *value);

//! Push list node.
void List_pushNode (
	List *l,
	ListNode *ln);

//! Determine if a given void value is an element of a given list
int List_isElement (
	const List *l,
	const void *v,
	int (*compare)(const void *v1, const void *v2));

//! Uniqueify list given comparrison function
void List_uniqueify (
	List *l,
	int (*compare)(const void *v1, const void *v2));

//! Insert an object into a sorted list given some comparison operator
void List_insertSorted (
	List *l,
	void *v,
	int (*isGreater)(const void *v1, const void *v2));

//! Delete elements from a list that match a given element
void List_deleteElements (
	List *l,
	const int value,
	int (*isMatch)(const int value, const void *v),
	const int deallocate);

//! Delete specific element from list
void List_deleteElement (
	List *l,
	ListNode **element,
	const int pointToNext,
	const int deallocate);

//! Delete element fro list, and call provided deallocate function.
void List_deleteElementWithDeallocator (
	List *l,
	ListNode **element,
	int pointToNext,
	void (*deallocator)(void *v));

//! Remove references to node from list.
void List_removeElement (
	List *l,
	const ListNode *element);

void List_shallowCopyNodes (List *dest, List *src);

#endif // LIST_H
