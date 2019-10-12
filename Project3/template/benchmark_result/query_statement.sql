/* Average for RRT in Cubicles & Twistycool */
SELECT AVG(time) AS TIME, AVG(solution_length) AS PATH_LENGTH, AVG(graph_states) AS GRAPH_STATES FROM runs WHERE plannerid == 1;

/* Average for EST in Cubicles & Twistycool */
SELECT AVG(time) AS TIME, AVG(solution_length) AS PATH_LENGTH, AVG(graph_states) AS GRAPH_STATES FROM runs WHERE plannerid == 2;

/* Average for PRM in Cubicles & Twistycool */
SELECT AVG(time) AS TIME, AVG(solution_length) AS PATH_LENGTH, AVG(graph_states) AS GRAPH_STATES FROM runs WHERE plannerid == 3;

/* Average for RTP in Cubicles & Twistycool */
SELECT AVG(time) AS TIME, AVG(solution_length) AS PATH_LENGTH, AVG(graph_states) AS GRAPH_STATES FROM runs WHERE plannerid == 4;