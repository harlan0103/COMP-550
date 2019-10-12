Input: a set of n vertices and edges
Create map[][] as a record distance between two vertices
Create dist[] as shortes distance 
Create visted[] as visted vertices
for i from 0 to n vertices do
    dist[i] = map[0][i]
visted[0] = true

for i from 1 to n vertices do
    Create min to MAX_VALUE as INFINITY
    for j from 0 to n vertices do  
        if !visted[i] && dist[j] < min
            set min = dist[j]
    visited[j] = true

    for k from 0 to m connected edges do
        if !visited[k] && dist[k] > dist[j] + map[j][k]
            dist[k] = dist[j] + map[j][k]



