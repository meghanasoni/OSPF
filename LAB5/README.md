Run the following command:

python3 OSPF.py -i ID -f infile -o outfile -h HELLO_INTERVAL -a LSA_INTERVAL -s SPF_INTERVAL


Note:
1. outfile should be in the format output-ID.txt
2. The links are assumed to be bidirectional and the routers are forwarding LSA packets to only their neighbors(not ALL routers) expect for the one which sent the LSA packet. 
3. For the example input, in order to check the accuracy of results, I kept the minc and maxc values for each edge to be equal. However, randomization is also demonstrated in the report examples. 
4. The attached screenshot is the shortest path tree for the given input graph(for ROUTER 0). It can be used to conveniently cross-check the results.
5. The standard code for Dijkstra's algorithm is taken from geeksforgeeks.com 
