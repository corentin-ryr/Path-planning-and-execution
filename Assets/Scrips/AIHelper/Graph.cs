using System.Collections;
using System.Collections.Generic;
using UnityEngine;


public class Node
{
    public Vector3 position;

    public List<Node> neighbors;

    public Node(Vector3 position)
    {
        this.position = position;
        neighbors = new List<Node>();
    }
}
