using System.Collections.Generic;
using UnityEngine;

public static class AStar
{

    public static Node[] ComputeShortestPath(Node startNode, Node goalNode)
    {
        PriorityQueue<Node> discoveredNodes = new PriorityQueue<Node>();
        discoveredNodes.Insert(startNode, 0);

        Dictionary<Node, Node> previousNodes = new Dictionary<Node, Node>();

        Dictionary<Node, float> pathCost = new Dictionary<Node, float>();
        Dictionary<Node, float> heurCost = new Dictionary<Node, float>();

        pathCost.Add(startNode, 0);
        heurCost.Add(startNode, Vector3.Distance(startNode.position, goalNode.position));

        while (discoveredNodes.Count() != 0)
        {
            Node currentNode = discoveredNodes.Pop();

            if (currentNode == goalNode)
            {
                // Reconstruct the path
                return reconstructPath(previousNodes, goalNode);
            }

            foreach (Node node in currentNode.neighbors)
            {
                float currentCost = pathCost[currentNode] + Vector3.Distance(node.position, currentNode.position); //Add 1 as it is the cost of a transition between two nodes
                if (!pathCost.ContainsKey(node) || currentCost < pathCost[node])
                {
                    previousNodes[node] = currentNode;
                    pathCost[node] = currentCost;
                    heurCost[node] = currentCost + Vector3.Distance(node.position, goalNode.position);

                    discoveredNodes.Insert(node, heurCost[node]);
                }
            }
        }

        return null;

    }

    private static Vector2Int[] getNeighbors(Vector2Int node, float[,] traversability)
    {
        //Neighbors in 4-connectivity
        List<Vector2Int> neighbors = new List<Vector2Int>();

        for (int i = 0; i < 8; i++)
        {
            if (i % 2 == 0) // Neighbors in 4-connexity
            {
                int j = i / 2;
                Vector2Int neighbor = j % 2 == 0 ? new Vector2Int(node.x + j - 1, node.y) : new Vector2Int(node.x, node.y + j - 2);

                if (isValidPosition(neighbor, traversability))
                {
                    neighbors.Add(neighbor);
                }
            }
            else // Neighbor in diagonal
            {
                // We check if the neighbors +1 and -1 are valid and add the diagonal neighbor if so (for the diagoal to be valid, the two non-diagonal paths must be clear)
                int jLeft = ((i - 1) % 8) / 2;
                Vector2Int neighborLeft = jLeft % 2 == 0 ? new Vector2Int(node.x + jLeft - 1, node.y) : new Vector2Int(node.x, node.y + jLeft - 2);
                int jRight = ((i + 1) % 8) / 2;
                Vector2Int neighborRight = jRight % 2 == 0 ? new Vector2Int(node.x + jRight - 1, node.y) : new Vector2Int(node.x, node.y + jRight - 2);

                Vector2Int neighbor = new Vector2Int(node.x + jLeft % 2 - 1, node.y + jLeft / 2 - 1);

                if (isValidPosition(neighborLeft, traversability) && isValidPosition(neighborRight, traversability) && isValidPosition(neighbor, traversability))
                {
                    neighbors.Add(neighbor);
                }
            }
        }

        return neighbors.ToArray();
    }

    private static bool isValidPosition(Vector2Int position, float[,] traversability)
    {
        if (position.x >= 0 && position.y >= 0 &&
            position.x < traversability.GetLength(0) && position.y < traversability.GetLength(1) &&
            traversability[position.x, position.y] == 0)
        {
            return true;
        }

        return false;
    }

    private static Node[] reconstructPath(Dictionary<Node, Node> previousNodes, Node currentNode)
    {
        List<Node> path = new List<Node>();
        path.Add(currentNode);

        while (previousNodes.ContainsKey(currentNode))
        {
            currentNode = previousNodes[currentNode];
            path.Add(currentNode);
        }
        path.Reverse();
        return path.ToArray();
    }
}
