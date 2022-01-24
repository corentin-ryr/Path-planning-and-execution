using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using FibonacciHeap;

public class PriorityQueue<T>
{

    private FibonacciHeap<T, float> heap;


    public PriorityQueue()
    {
        heap = new FibonacciHeap<T, float>(0);
    }

    /// Inserts and item with a priority
    public void Insert(T item, float priority)
    {
        heap.Insert(new FibonacciHeapNode<T, float>(item, priority));
    }

    /// Returns the element with the highest priority
    public T Top()
    {
        return heap.Min().Data;
    }

    /// Deletes and returns the element with the highest priority
    public T Pop()
    {
        return heap.RemoveMin().Data;
    }

    public int Count()
    {
        return heap.Size();
    }

}

