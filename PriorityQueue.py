#Thanks CS 3600 TAs...
import heapq
class PriorityQueue:
    def  __init__(self):
        self.heap = []
        self.count = 0
        self.size = 0

    def push(self, item, priority):
        entry = (priority, self.count, item)
        heapq.heappush(self.heap, entry)
        self.count += 1
        self.size += 1

    def pop(self):
        (_,_ , item) = heapq.heappop(self.heap)
        self.size -= 1
        return item

    def empty(self):
        return self.size == 0
