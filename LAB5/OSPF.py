# NAME: Meghana Gopal Soni
# Roll Number: CS20B051
# Course: CS3205 Jan. 2023 semester
# Lab number: 5
# Date of submission: 28-04-2023
# I confirm that the source file(except Dijkstra's algorithm function) is entirely written by me without resorting to any dishonest means.
# Website(s) that I used for basic socket programming code are:
# URL(s): https://pythontic.com/modules/socket/udp-client-server-example, https://www.geeksforgeeks.org/python-program-for-dijkstras-shortest-path-algorithm-greedy-algo-7/

import socket
import multiprocessing 
import threading
import time
import random
import sys, getopt
import fileinput
from tabulate import tabulate
localIP     = "127.0.0.1"
bufferSize  = 1024


HELLO_INTERVAL = 1
LSA_INTERVAL = 5
SPF_INTERVAL = 20
"""DIJKSTRA'S CODE FROM GFG"""
class Graph():
 
    def __init__(self, vertices):
        self.V = vertices
        self.graph = [[1e9 for column in range(vertices)]
                      for row in range(vertices)]
    def create_path(self, prev, j, src):
    	s = ""
    	if(j == src):
    		return str(j)
    	if(prev[j]==-1):
    		return s
    	return self.create_path(prev, prev[j], src) + "->" + str(j)
 
    def printSolution(self, dist, src, prev):
    	global ID, outfile
    	
    	table = []
    	f = open("output-" + str(src) + ".txt", "a")
    	
    	for node in range(self.V):
    		l = []
    		path = self.create_path(prev, node, src)
    		
    		l.append(str(node))
    		l.append(path)
    		l.append(str(dist[node]))
    		table.append(l)
    	
    	
    	
    	f.write(tabulate(table, headers=["Destination", "Path", "Cost"]))
    	f.write("\n")
    	
 
    def minDistance(self, dist, sptSet):
 
        min = 1e9
        for v in range(self.V):
            if dist[v] < min and sptSet[v] == False:
                min = dist[v]
                min_index = v
 
        return min_index
 
    def dijkstra(self, src):
 
        dist = [1e7] * self.V
        dist[src] = 0
        sptSet = [False] * self.V
        prev = [-1]*self.V
        prev[src] = -1
        for cout in range(self.V):
 
            u = self.minDistance(dist, sptSet)
 
            # Put the minimum distance vertex in the
            # shortest path tree
            sptSet[u] = True
 
            for v in range(self.V):
                if (self.graph[u][v] > 0 and
                   sptSet[v] == False and
                   dist[v] > dist[u] + self.graph[u][v]):
                    dist[v] = dist[u] + self.graph[u][v]
                    prev[v] = u
 
        self.printSolution(dist, src, prev)
        
  
def hello_packets(i, UDPServerSocket):    
	global neighbors, localIP, HELLO_INTERVAL, cost_range, cost
	#Send hello message to all neighbors every HELLO_INTERVAL seconds
	while(1):
		for j in neighbors[i]:
			message = "HELLO|" + str(i)  #send HELLO|srcID to neighbor with port number 10000+j
			#print("sent message " + message + " from " + str(i) + " to " + str(j))
			UDPServerSocket.sendto(message.encode(), (localIP, 10000+j)) 
		time.sleep(HELLO_INTERVAL)
	
def receive_packets(i, UDPServerSocket, topology_graph):
	global LSA_seq_no, cost, cost_range
	while(1):
		# Listen for incoming datagrams
		bytesAddressPair = UDPServerSocket.recvfrom(bufferSize)
		message = bytesAddressPair[0].decode()
		address = bytesAddressPair[1]
		#print("I am router " + str(i) + " and I received message from a neighbor : " + str(message))
		l = message.split("|") #Note that message has the format HELLO|j or HELLOREPLY|i|j|c(ij) 
		if(l[0]=="HELLO"):
			src = int(l[1])
			(minc, maxc) = cost_range[(src, i)]
			random_cost = random.randint(minc, maxc)
			new_msg = "HELLOREPLY|" + str(i) + "|" + str(src) + "|"+ str(random_cost)
			UDPServerSocket.sendto(new_msg.encode(), (localIP, 10000+src)) 
		elif(l[0]=="HELLOREPLY"):
		 #received a hello reply message, so change set link value 
			j = int(l[1])
			i = int(l[2])
			c = int(l[3])
			cost[(i, j)] = c
			topology_graph.graph[i][j] = c
			#("COST OF LINK FROM "+str(i) + " to " + str(j) + " is " + str(c))
		else:	
			"""f = open("LSA-" + str(i) + ".txt", "a")
			f.write(message)
			f.write("\n")"""
			#LSA message received
			l = message.split("|")
			srcid = int(l[1])
			seqno = int(l[2])
			entries = int(l[3])
			if(seqno <= LSA_seq_no[i][srcid]):
				continue
			for j in range(4, 3 + 2*entries, 2):
		
				x = int(l[j])
				c = int(l[j+1])
				topology_graph.graph[srcid][x] = c
				
			LSA_seq_no[i][srcid] = seqno
			for j in neighbors[i]:
				if(j != srcid):
					UDPServerSocket.sendto(message.encode(), (localIP, 10000+j))
		

def LSA(i, UDPServerSocket):
	global neighbors, localIP, LSA_INTERVAL, cost
	sequence_number = 0
	while(1):
		message = "LSA|" + str(i) + "|" + str(sequence_number) + "|" + str(len(neighbors[i])) 
		
		for j in neighbors[i]:
			message += "|" + str(j) + "|" + str(cost[(i, j)])
		for j in neighbors[i]:
			UDPServerSocket.sendto(message.encode(), (localIP, 10000+j)) 
		#print("LSA Message from " + str(i) + " is " + message)
		sequence_number+=1
		time.sleep(LSA_INTERVAL)
		
def routing_table_construction(i, topology_graph):
	global SPF_INTERVAL, init
	
	while(1):
		f = open("output-" + str(i) + ".txt", "a")
		"""for k in range(N):
			for l in range(N):
				f.write((str(topology_graph.graph[k][l]) +   " "))
			f.write("\n")"""
		
		f.write(str("Routing Table for Node No. " + (str(i))  +" at Time " + str(int(time.time()-init)) + "\n"))
		lock = multiprocessing.Lock()
		# acquire the lock
		lock.acquire()
		topology_graph.dijkstra(i)
		lock.release()
		time.sleep(SPF_INTERVAL)
	
def router_process(i):
	# Create a datagram socket
	global N
	UDPServerSocket = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
	topology_graph = Graph(N)
	f = open("output-" + str(i) + ".txt", "w")
	f.close()
	"""f = open("LSA-" + str(i) + ".txt", "w")
	f.close()"""
	# Bind to address and ip
	UDPServerSocket.bind((localIP, 10000+i))
	
	helloThread = threading.Thread(target=hello_packets, args=(i, UDPServerSocket))
	LSAThread = threading.Thread(target = LSA, args = (i, UDPServerSocket))
	receptionThread = threading.Thread(target = receive_packets, args = (i, UDPServerSocket, topology_graph))
	routingTableThread = threading.Thread(target = routing_table_construction, args = (i, topology_graph))
	helloThread.start()
	LSAThread.start()
	receptionThread.start()
	routingTableThread.start()
	helloThread.join()
	LSAThread.join()
	receptionThread.join()
	
"""./ospf -i id -f infile -o outfile -h hi -a lsai -s spfi"""

options = "i:f:o:h:a:s:"
argumentList = sys.argv[1:]
infile = "infile.txt"
outfile = "outfile.txt"
ID = 3
arguments, values = getopt.getopt(argumentList, options, [])
for currentArgument, currentValue in arguments:
	if currentArgument == "-i":
		ID = currentValue
	elif currentArgument == "-f":
		infile = currentValue
	elif currentArgument == "-o":
		outfile = currentValue
	elif currentArgument == "-h":
		HELLO_INTERVAL = int(currentValue)
	elif currentArgument == "-a":
		LSA_INTERVAL = int(currentValue)
	elif currentArgument == "-s":
		SPF_INTERVAL = int(currentValue)
f = open(outfile, "w")
f.close()	
input_list = []
for line in fileinput.input(files =infile):
	input_list.append(line)
#Take input. The dictionary cost_range stores [(i, j)]: [(minc, maxc)]
#N,e = input().split()
N, e = input_list[0].split()
e = int(e)
N = int(N)
cost_range = {}
neighbors = {}
cost = {}
LSA_seq_no = [[-1]*N]*N
#initialize the neighbor list for each router to NULL
for i in range(N):
	neighbors[i] = []

init = time.time()	

#for every edge (i, j) in E , cost_range dictionary has (i, j) ---> (minc, maxc)
for x in range(e):
	#i, j, minc, maxc = input().split()
	i, j, minc, maxc = input_list[x+1].split()
	i = int(i)
	j = int(j)
	neighbors[i].append(j)
	neighbors[j].append(i)
	cost[(i, j)] = 1000000000
	cost[(j, i)] = 1000000000
	minc = int(minc)
	maxc = int(maxc)
	cost_range[(i, j)] = (minc, maxc)
	cost_range[(j, i)] = (minc, maxc)

#initialize N processes and store them in the array processes
processes = []
for i in range(N):
	proc = multiprocessing.Process(target=router_process, args=(i, ))
	processes.append(proc)

#start all processes
for i in range(N):
	processes[i].start()
	
for i in range(N):
	processes[i].join()

