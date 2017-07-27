import heapq

STATUS_NO = 0
STATUS_OPEN = 1
STATUS_CLOSE = 2

DIR_8_LIST = [
    [1,-1],
    [1, 0],
    [1, 1],
    [0,-1],
    [0, 1],
    [-1,-1],
    [-1,0,],
    [-1,1],
]

DIR_4_LIST = [
    [-1,0],
    [1, 0],
    [0,-1],
    [0, 1],
]

COLOR_DESC = {
    'black':30,
    'red':31,
    'yellow':33,
    'blue':34,
    'green':32,
    'purple':35,
    'cyan':36,
    'white':37,
}

def color_pack(s, color):
    return "\033[%dm%s\033[0m"%(COLOR_DESC.get(color,37),s)


class CNode:
    def __init__(self, row, col):
        self.m_Row = row
        self.m_Col = col
        self.m_Mark = '.'
        self.m_Status = STATUS_NO
        self.m_Parent = None
        self.m_gScore = 0
        self.m_fScore = 0

    def restore(self):
        self.m_Status = STATUS_NO
        self.m_Parent = None
        self.m_gScore = 0
        self.m_fScore = 0

    def is_goalnode(self):
        return self.m_Mark == 'g'

    def is_startnode(self):
        return self.m_Mark == 's'

    def is_open(self):
        return self.m_Status == STATUS_OPEN

    def is_close(self):
        return self.m_Status == STATUS_CLOSE


class CMap:
    def __init__(self, map_list):
        self.m_StartPos = None
        self.m_GoalPos = None
        self.m_MaxCol = len(map_list[0])
        self.m_MaxRow = len(map_list)
        self.m_NodesList = [[None]*self.m_MaxCol for i in range(self.m_MaxRow)]
        for i in xrange(self.m_MaxRow):
            for j in xrange(self.m_MaxCol):
                mark = map_list[i][j]
                if mark != '1':
                    self.m_NodesList[i][j] = CNode(i,j)
        print color_pack("init map[%dx%d] ok!"%(self.m_MaxRow,self.m_MaxCol), "yellow")

    def set_config(self, stapos, goalpos, diagonal_able=False, max_depth=None):
        last_sta_node = self.get_start_node()
        if last_sta_node:
            last_sta_node.m_Mark = '.'
        self.m_StartPos = stapos
        start_node = self.get_start_node()
        start_node.m_Mark = 's'

        last_goal_node = self.get_goal_node()
        if last_goal_node:
            last_goal_node.m_Mark = '.'
        self.m_GoalPos = goalpos
        goal_node = self.get_goal_node()
        goal_node.m_Mark = 'g'

        if max_depth:
            self.m_MaxDepth = max_depth
        else:
            self.m_MaxDepth = self.m_MaxCol*self.m_MaxRow
        self.m_DiagonalAble = diagonal_able

    def get_start_node(self):
        if self.m_StartPos:
            sr,sc = self.m_StartPos
            return self.m_NodesList[sr][sc]

    def get_goal_node(self):
        if self.m_GoalPos:
            gr,gc = self.m_GoalPos
            return self.m_NodesList[gr][gc]

    def is_block(self, r, c):
        if (0<=r<self.m_MaxRow and 0<=c<self.m_MaxCol) and (self.m_NodesList[r][c]!=None):
            return False
        return True

    def compute_g(self, node, parent):
        if parent:
            if self.m_DiagonalAble and (abs(node.m_Row-parent.m_Row)+abs(node.m_Col-parent.m_Col) > 1):
                return parent.m_gScore + 14
            else:
                return parent.m_gScore + 10
        return 0

    def compute_h(self, node):
        gr,gc = self.m_GoalPos
        dr = abs(node.m_Row - gr)
        dc = abs(node.m_Col - gc)
        if self.m_DiagonalAble:
            if dr > dc: # 1:1:sqrt(2)
                return (dr-dc)*10 + 14*dc
            else:
                return (dc-dr)*10 + 14*dr
        else:
            return 10*(dr+dc)

    def add_open(self, node, parent=None):
        self.m_CurDepth = self.m_CurDepth + 1
        node.m_Status = STATUS_OPEN
        node.m_Parent = parent
        node.m_gScore = self.compute_g(node, parent)
        node.m_hScore = self.compute_h(node)
        node.m_fScore = node.m_gScore + node.m_hScore
        heapq.heappush(self.m_OpenList, (node.m_fScore,node))
        if self.m_CurDepth >= self.m_MaxDepth:
            goal_node = self.get_goal_node()
            if not goal_node in self.m_OpenList:
                return True

    def add_close(self, node):
        node.m_Status = STATUS_CLOSE
        self.m_CloseList.append(node)

    def find_path(self): #A* pathfinding
        self.m_OpenList = []
        self.m_CloseList = []
        self.m_CurDepth = 0
        dir_list = None
        if self.m_DiagonalAble:
            dir_list = DIR_8_LIST
        else:
            dir_list = DIR_4_LIST
        for i in xrange(self.m_MaxRow):
            for j in xrange(self.m_MaxCol):
                if self.m_NodesList[i][j] != None:
                    self.m_NodesList[i][j].restore()
        start_node = self.get_start_node()
        self.add_open(start_node)
        while len(self.m_OpenList) > 0:
            current = heapq.heappop(self.m_OpenList)[1]
            if current.is_goalnode():
                self.reconstruct_path(current)
                return
            self.add_close(current)
            for d in dir_list:
                nr = d[0] + current.m_Row
                nc = d[1] + current.m_Col
                if self.is_block(nr, nc):
                    continue
                neighbor = self.m_NodesList[nr][nc]
                if neighbor.is_close():
                    continue
                if neighbor.is_open():
                    tentative_gscore = self.compute_g(neighbor, current)
                    if tentative_gscore < neighbor.m_gScore:
                        neighbor.m_Parent = current
                        neighbor.m_gScore = tentative_gscore
                        neighbor.m_fScore = tentative_gscore + neighbor.m_hScore
                else:
                    if self.m_CurDepth < self.m_MaxDepth:
                        if self.add_open(neighbor, current):
                            lastnode = self.m_CloseList[-1]
                            self.reconstruct_path(lastnode)
                            return
        self.reconstruct_path(None)

    def reconstruct_path(self, lastnode):
        map_list = [[color_pack('0','green')]*self.m_MaxCol for i in range(self.m_MaxRow)]
        for i in xrange(self.m_MaxRow):
            for j in xrange(self.m_MaxCol):
                if self.m_NodesList[i][j] != None:
                    if self.m_NodesList[i][j].is_startnode():
                        map_list[i][j] = color_pack('s','blue')
                    elif self.m_NodesList[i][j].is_goalnode():
                        map_list[i][j] = color_pack('g','yellow')
                else:
                    map_list[i][j] = color_pack('1','red')
        msg = "%d dir search cost depth:%d limit depth:%d "%(8 if self.m_DiagonalAble else 4, self.m_CurDepth, self.m_MaxDepth)
        if lastnode == None:
            msg = msg + color_pack("result:can,t access[failed]", "black")
        else:
            current = lastnode
            if lastnode.is_goalnode():
                msg = msg + color_pack("result:get it[succeed]", "blue")
            else:
                msg = msg + color_pack("result:depth limit[failed]", "cyan")
            while current:
                if not current.is_startnode() and not current.is_goalnode():
                    map_list[current.m_Row][current.m_Col] = color_pack("*",'green')
                current = current.m_Parent
        print msg
        for v in map_list:
            print " ".join(v)


map_data = [
    "1....1111",
    "..1111...",
    "1...111.1",
    "...1...1.",
    "...11....",
    "1..1.1111",
    "1....1..1",
]

pf = CMap(map_data)
#4 dir search
pf.set_config((1,1),(5,4))
pf.find_path()
#8 dir search
pf.set_config((1,1),(1,6), True)
pf.find_path()
#depth limit search
pf.set_config((1,1),(1,6), True, 20)
pf.find_path()
#cant access
pf.set_config((1,1),(1,6))
pf.find_path()
pf.set_config((1,1),(6,6),True)
pf.find_path()