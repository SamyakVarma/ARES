<<<<<<< HEAD
<<<<<<< HEAD
import sys
from heapq import heapify,heappush,heappop

graph={
    'A':{'B':2,'C':4},
    'B':{'A':2,'C':3,'D':8},
    'C':{'A':4,'C':3,'E':5,'D':2},
    'D':{'B':8,'C':2,'E':11,'F':22},
    'E':{'C':5,'D':11,'F':1},
    'F':{'D':22,'E':1}
}
start='A'
end='F'

def dijkastra(graph,start,fin):
    inf=sys.maxsize
    nodes={}
    visited=[]
    for n in graph:
        newNode={n:{'cost':inf,'pred':[]}}
        nodes.update(newNode)
    nodes[start]['cost']=0
    curr_node=start
    for i in range(len(graph)-1):
        if curr_node not in visited:
            visited.append(curr_node)
            min_heap=[]
            for node in graph[curr_node]:
                if node not in visited:
                    cost= nodes[curr_node]['cost']+graph[curr_node][node]
                    if cost<nodes[node]['cost']:
                        nodes[node]['cost']=cost
                        nodes[node]['pred']=nodes[curr_node]['pred']+list(curr_node)
                    heappush(min_heap,(nodes[node]['cost'],node))
        heapify(min_heap)
        curr_node=min_heap[0][1]
    print("Shortest dist = "+ str(nodes[fin]['cost']))
    print("Shortest path = "+ str(nodes[fin]['pred']+list(fin)))
=======
=======
>>>>>>> f0e7726ebbc9fc0fb28137e81fb25a72f067c579
import sys
from heapq import heapify,heappush,heappop

graph={
    'A':{'B':2,'C':4},
    'B':{'A':2,'C':3,'D':8},
    'C':{'A':4,'C':3,'E':5,'D':2},
    'D':{'B':8,'C':2,'E':11,'F':22},
    'E':{'C':5,'D':11,'F':1},
    'F':{'D':22,'E':1}
}
start='A'
end='F'

def dijkastra(graph,start,fin):
    inf=sys.maxsize
    nodes={}
    visited=[]
    for n in graph:
        newNode={n:{'cost':inf,'pred':[]}}
        nodes.update(newNode)
    nodes[start]['cost']=0
    curr_node=start
    for i in range(len(graph)-1):
        if curr_node not in visited:
            visited.append(curr_node)
            min_heap=[]
            for node in graph[curr_node]:
                if node not in visited:
                    cost= nodes[curr_node]['cost']+graph[curr_node][node]
                    if cost<nodes[node]['cost']:
                        nodes[node]['cost']=cost
                        nodes[node]['pred']=nodes[curr_node]['pred']+list(curr_node)
                    heappush(min_heap,(nodes[node]['cost'],node))
        heapify(min_heap)
        curr_node=min_heap[0][1]
    print("Shortest dist = "+ str(nodes[fin]['cost']))
    print("Shortest path = "+ str(nodes[fin]['pred']+list(fin)))
<<<<<<< HEAD
>>>>>>> f0e7726e (updated face recognitions)
=======
>>>>>>> f0e7726ebbc9fc0fb28137e81fb25a72f067c579
dijkastra(graph,start,end)