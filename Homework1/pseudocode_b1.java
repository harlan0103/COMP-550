Input: a set of vertices n, and a set of n obstacle boundary edges
for v1 in n vertices do
    for v2 in n vertices do
        if v1 != v2 then we have pair<v1, v2>
        for e in n boundary edges do
            if <v1, v2> is intersects with boundary edge e
                return
            else
                add <v1, v2> as a new edge



