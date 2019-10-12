void computeIntersectionPoint(pair<A1, B1>, pair<A2, B2>):
    // We first check if two line segments are parallel
    if computeCrossProduct((A1.x - B1.x), (A2.y - B2.y), (A1.y - B1.y), (A1.x - B2.x)) == 0
        // We need to check if two line is collinear
        // Collinear means two lines may lie on the same line
        // We chose one point from each segment and calculate cross product with one segment line
        if computeCrossProduct((A1.x - B1.x), (A1.y, B1.y), (B1.x - A2.x), (B1.y - A2.y)) == 0
            // CrossProduct is 0 means two lines are collinear
            if(compareTwoNode(A1, A2) <= 0 && compareTwoNode(B1, B2) >= 0)
                // We have A1 ---- A2 ---- B1 ---- B2
                return A2
            else if(compareTwoNode(A1, A2) >= 0 && compareTwoNode(B1, B2) <= 0)
                // We have A2 ---- A1 ---- B2 ---- B1
                return A1
            else
                return 'no intersection'
        else
            // Since two lines are not collinear, then there is not intersection
            return 'no intersection'
    else
        // Two lines are not parallel, then they may have intersection
        if compareTwoNode(A1, A2) == 0 || compareTwoNode(A1, B2) == 0
            // Two lines intersect at A1
            return A1
        else if compareTwoNode(B1, A2) == 0 || compareTwoNode(B1, B2) == 0
            // Two lines intersect at B1
            return B1
        else 
            // Two lines intersect on a point which is not the four endpoints
            // We know that for point (x1,y1), (x2,y2)
            // The line equation is (x - x1) / (x2 - x1) = (y - y1) / (y2 - y1)
            // Then x = t(x2 - x1) + x1 and y = s(y2 - y1) + y1
            // Thus the intersect point is (t(B1.x - A1.x) + A1.x , s(B1.y - A1.y) + A1.y)
            return (t(B1.x - A1.x) + A1.x, s(B1.y - A1.y) + A1.y)


// Compare two node
int compareTwoNode(node a, node b):
    if a.x < b.x || a.x == b.x && a.y < b.y
        return -1
    else if a.x > b.x || a.x == b.x && a.y > b.y
        return 1
    else if a.x == b.x && a.y == b.y
        // Two points are the same
        return 0


// CrossProduct = x1 * y2 - y1 * x2
// If crossProduct is 0, then two lines are parallel
void computeCrossProduct(point x1, point y1, point x2, point y2):
    return x1 * y2 - y1 * x2