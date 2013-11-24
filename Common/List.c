
#include "List.h"

ListNode initListNode()
{
	ListNode n;
	n.value = 0;
	n.prev = 0;
	n.next = 0;
	return n;
}

void ListNode_clear (ListNode *ln,
					 const int deallocate)
{
	if (deallocate)
	{
		free (ln->value);
	}
}

void ListNode_clearWithDeallocator (ListNode *ln,
									void (*deallocate)(void *v))
{
	deallocate (ln->value);
	free (ln->value);
}

List initList()
{
	List l;
	l.size = 0;
	l.front = 0;
	l.back = 0;
	return l;
}

void List_clear (List *l, const int deallocate)
{
	ListNode *temp;
	ListNode *iterator;
	iterator = l->back;
	while (iterator)
	{
		temp = iterator;
		iterator = iterator->prev;

		if (deallocate)
		{
			free (temp->value);
		}
		free (temp);
	}
	l->front = l->back = 0;
	l->size = 0;
}

void List_clearWithDeallocator (List *l, void (*deallocator)(void *v))
{
	ListNode *temp;
	ListNode *iterator;
	iterator = l->back;
	while (iterator)
	{
		temp = iterator;
		iterator = iterator->prev;

		ListNode_clearWithDeallocator (temp, deallocator);
		free (temp);
	}
	l->front = l->back = 0;
	l->size = 0;
}

void List_pushValue (List *l, void *value)
{
	ListNode *node = (ListNode*)malloc (sizeof (ListNode));
	*node = initListNode();
	node->value = value;
	List_pushNode (l, node);
}

void List_pushNode (List *l,
					ListNode *ln)
{
	// If this is the first element, set the front pointer to point to this
	if (0 == l->size)
	{
		l->front = ln;

		ln->prev = 0;
	}
	else
	{
		// Make sure that the back pointer is ok
		RELEASE_ASSERT(l->back)

		// gGet new element and last element to point to eachother
		l->back->next = ln;
		ln->prev = l->back;
	}

	ln->next = 0;
	l->back = ln;

	++l->size;
}

int List_isElement (const List *l,
					const void *v,
					int (*compare)(const void *v1, const void *v2))
{
	ListNode *iterator;
	iterator = l->front;
	while (iterator != 0)
	{
		if (1 == compare (iterator->value, v))
		{
			return 1;
		}
		iterator = iterator->next;
	}
	return 0;
}

void List_uniqueify (
	List *l,
	int (*compare)(const void *v1, const void *v2))
{
	ListNode *iter, *iter2;

	iter = l->front;
	while (iter)
	{
		iter2 = l->front;
		while (iter2)
		{
			if (iter2 != iter && 1 == compare (iter->value, iter2->value))
			{
				List_deleteElement (l, &iter2, 1, 1);
			}
			else
			{
				iter2 = iter2->next;
			}
		}

		iter = iter->next;
	}
}

//! Insert a point in the list in front of another given point
void insertBefore (List *l,
				   ListNode *insert,
				   ListNode *element)
{
	if (0 == l->size || 0 == element)
	{
		List_pushNode (l, insert);
		return;
	}

	insert->next = element;
	insert->prev = element->prev;
	element->prev = insert;

	if (element == l->front)
	{
		l->front = insert;
	}
	++l->size;
}

void List_insertSorted (List *l,
						void *v,
						int (*isGreater)(const void *v1, const void *v2))
{
	ListNode *iterator;
	ListNode *toInsert;

	if (0 == l->size)
	{
		List_pushValue (l, v);
		return;
	}

	iterator = l->front;
	while (1)
	{
		if (0 == iterator)
		{
			List_pushValue (l, v);
			return;
		}
		
		if (isGreater (v, iterator->value))
		{
			toInsert = (ListNode*)malloc (sizeof (ListNode));
			*toInsert = initListNode();
			toInsert->value = v;

			insertBefore (l, toInsert, iterator);
			return;
		}

		iterator = iterator->next;
	}
}

void List_deleteElements (List *l,
						  const int value,
						  int (*isMatch)(const int value, const void *v),
						  const int deallocate)
{
	ListNode *iterator;
	iterator = l->front;
	while (iterator)
	{
		if (1 == isMatch (value, iterator->value))
		{
			List_deleteElement (
				l,
				&iterator,
				1,
				deallocate);
		}
		// If the element at iterator is deleted, the iterator
		// will be made to point to the next element in the list.
		else
		{
			iterator = iterator->next;
		}
	}
}

void List_deleteElement (List *l,
						 ListNode **element,
						 const int pointToNext,
						 const int deallocate)
{
	ListNode *temp = *element;

	// Remove all reference to it in the list
	List_removeElement (l, *element);

	// Set the pointer to point to next or previous element.
	if (1 == pointToNext)
	{
		*element = temp->next;
	}
	else
	{
		*element = temp->prev;
	}

	ListNode_clear (temp, deallocate);
	free (temp);
}

void List_deleteElementWithDeallocator (List *l,
										ListNode **element,
										int pointToNext,
										void (*deallocator)(void *v))
{
	ListNode *temp = *element;

	// Remove all reference to it in the list
	List_removeElement (l, *element);

	// Set the pointer to point to the either the next 
	// element in the list or the previous element
	if (1 == pointToNext)
	{
		*element = temp->next;
	}
	else
	{
		*element = temp->prev;
	}

	// delete the element
	ListNode_clearWithDeallocator (temp, deallocator);
	free (temp);
}


//! Remove all pointers to element in list
void List_removeElement (List *l,
						 const ListNode *element)
{
	// change the list pointers, if there is only one element in the list, the 
	// front and back pointers will be given values of 0
	if (l->front == element)
	{
		l->front = element->next;

		if (l->front)
		{
			l->front->prev = 0;
		}
	}

	if (l->back == element)
	{
		l->back = element->prev;

		if (l->back)
		{
			l->back->next = 0;
		}
	}

	// change the neighbours pointers
	if (element->prev)
		element->prev->next = element->next;

	if (element->next)
		element->next->prev = element->prev;

	--l->size;
}

void List_shallowCopyNodes (List *dest, List *src)
{
	ListNode *iterator;
	ListNode *temp;

	iterator = src->front;
	while (iterator)
	{
		temp = iterator;
		iterator = iterator->next;

		List_removeElement (src, temp);
		List_pushNode (dest, temp);
	}
}



