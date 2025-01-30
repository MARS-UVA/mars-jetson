#include <cassert>
#include <iostream>
#include "../src/models/pc_adacency_tree.h"

void testPointcloudTreeBasicInsertion()
{
    PointcloudTree tree(Point(-5, 10), Point(5, 0));

    Vertex *v = new Vertex(0, 0, 0);
    tree.add(v);
    Node searchNode(Point(0, 0), 0);
    assert(tree.find(&searchNode) != nullptr);

    Node *a = new Node(Point(1.0, 1.0), 1);
    Node *b = new Node(Point(2, 5), 2);
    Node *c = new Node(Point(7, 6), 3);

    tree.add(a);
    tree.add(b);
    tree.add(c);

    tree.print(0);

    assert(tree.find(a) != nullptr);
    assert(tree.find(b) != nullptr);
    assert(tree.find(c) != nullptr);

    delete v;
    tree.clear();
    std::cout << "Basic insertion test passed!" << std::endl;
}

void testPointcloudTreeBoundaryConditions()
{
    PointcloudTree tree(Point(-5, 10), Point(5, 0));

    // Test inserting point outside boundary
    Vertex *outsidePoint = new Vertex(10, 15, 0);
    tree.add(outsidePoint);
    // This should not crash or cause undefined behavior

    Node *n = new Node(Point(outsidePoint->x, outsidePoint->y), outsidePoint->z);
    assert(tree.find(n) != nullptr);
    tree.clear();
    std::cout << "Boundary conditions test passed!" << std::endl;
}

void testPointcloudTreeDuplicateInsertion()
{
    PointcloudTree tree(Point(-5, 10), Point(5, 0));

    Node *a = new Node(Point(1.0, 1.0), 1);
    tree.add(a);
    tree.add(a);

    // Should print only one instance of the node
    tree.print(0);

    assert(tree.find(a) != nullptr);

    tree.clear();
    std::cout << "Duplicate insertion test passed!" << std::endl;
}

int main()
{
    testPointcloudTreeBasicInsertion();
    testPointcloudTreeBoundaryConditions();
    testPointcloudTreeDuplicateInsertion();

    std::cout << "All tests passed!" << std::endl;
    return 0;
}