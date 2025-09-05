import pygame
import random
import math
import heapq
import json
import collections

# --- Σταθερές ---
TILE_SIZE = 30
GRID_WIDTH_CELLS = 25
GRID_HEIGHT_CELLS = 20
SCREEN_WIDTH = GRID_WIDTH_CELLS * TILE_SIZE
SCREEN_HEIGHT = GRID_HEIGHT_CELLS * TILE_SIZE

# --- Παράμετροι Αποστολής (Default) ---
NUM_PATROL_MIN, NUM_PATROL_MAX = 3, 5
NUM_AGGRESSIVE_MIN, NUM_AGGRESSIVE_MAX = 2, 3

# --- Χρώματα & Τύποι Κελιών ---
BLACK, WHITE = (0, 0, 0), (255, 255, 255)
COLOR_FREE = (200, 200, 200)
COLOR_OBSTACLE = (50, 50, 50)
COLOR_DIFFICULT = (100, 100, 160)
COLOR_HAZARD = (255, 180, 180)
COLOR_WATER = (30, 90, 180)
COLOR_AGENT = (0, 120, 255)
COLOR_AQUATIC_AGENT = (0, 200, 200)
COLOR_GOAL_OUTLINE = (255, 215, 0)
COLOR_PATH = (0, 150, 0, 150)
COLOR_COLLECTIBLE = (255, 255, 0)
COLOR_PATROL_OBSTACLE, COLOR_AGGRESSIVE_OBSTACLE = (180, 0, 0), (255, 0, 255)
MENU_BG_COLOR = (10, 25, 47)
MENU_TEXT_COLOR = (200, 200, 255)
MENU_HIGHLIGHT_COLOR = (255, 255, 0)

TILE_TYPE_FREE = 0
TILE_TYPE_OBSTACLE = 1
TILE_TYPE_DIFFICULT = 2
TILE_TYPE_HAZARD = 3
TILE_TYPE_WATER = 4

# --- Κλάσεις ---
class Node:
    def __init__(self, position, parent=None):
        self.parent, self.position = parent, position
        self.g, self.h, self.f = 0, 0, 0
    def __eq__(self, other): return self.position == other.position
    def __lt__(self, other): return self.f < other.f
    def __hash__(self): return hash(self.position)

class Agent:
    def __init__(self, x, y):
        self.x, self.y, self.path, self.score = x, y, [], 0
        self.wait_timer = 0
    def set_path(self, path):
        if path and path[0] == (self.x, self.y):
            self.path = path[1:]
        elif path:
            self.path = path
        else:
            self.path = []
    def move(self):
        if self.wait_timer > 0: self.wait_timer -= 1; return
        if self.path and abs(self.path[0][0] - self.x) + abs(self.path[0][1] - self.y) == 1:
            self.x, self.y = self.path.pop(0)
        else:
            self.path = []
    def draw(self, surface):
        rect = pygame.Rect(self.x * TILE_SIZE, self.y * TILE_SIZE, TILE_SIZE, TILE_SIZE)
        pygame.draw.rect(surface, COLOR_AGENT, rect, 0, 4)

class PatrolObstacle:
    def __init__(self, x, y, game_map):
        self.x, self.y, self.game_map = x, y, game_map
    def update(self, allowed_tiles):
        moves = [(dx, dy) for dx, dy in [(0, 1), (0, -1), (1, 0), (-1, 0)] if 0 <= (self.x + dx) < GRID_WIDTH_CELLS and 0 <= (self.y + dy) < GRID_HEIGHT_CELLS and self.game_map[self.y + dy][self.x + dx] in allowed_tiles]
        if moves:
            self.x, self.y = random.choice([(self.x + dx, self.y + dy) for dx, dy in moves])
    def draw(self, surface):
        rect = pygame.Rect(self.x * TILE_SIZE, self.y * TILE_SIZE, TILE_SIZE, TILE_SIZE)
        pygame.draw.ellipse(surface, COLOR_PATROL_OBSTACLE, rect)

class AggressiveObstacle(PatrolObstacle):
    def __init__(self, x, y, game_map, agent):
        super().__init__(x, y, game_map)
        self.agent, self.detection_radius, self.mode = agent, 6, 'patrol'
    def update(self, allowed_tiles):
        dist = math.hypot(self.x - self.agent.x, self.y - self.agent.y)
        self.mode = 'attack' if dist <= self.detection_radius else 'patrol'
        if self.mode == 'attack':
            game = Game.get_instance()
            if game:
                path_res, _ = game.a_star((self.x, self.y), (self.agent.x, self.agent.y), [TILE_TYPE_OBSTACLE, TILE_TYPE_WATER])
                if path_res and len(path_res) > 1:
                    self.x, self.y = path_res[1]
                else:
                    super().update(allowed_tiles)
        else:
            super().update(allowed_tiles)
    def draw(self, surface):
        rect = pygame.Rect(self.x * TILE_SIZE, self.y * TILE_SIZE, TILE_SIZE, TILE_SIZE)
        pygame.draw.ellipse(surface, COLOR_AGGRESSIVE_OBSTACLE if self.mode == 'attack' else COLOR_PATROL_OBSTACLE, rect)

class Drone:
    def __init__(self, x, y, color=COLOR_AGENT, speed=0.4):
        self.x, self.y, self.path, self.vision_radius, self.speed, self.color = float(x), float(y), [], 4, speed, color
    def set_path(self, path):
        self.path = list(path)
    def move(self):
        if not self.path: return
        tx, ty = self.path[0]
        dx, dy = tx - self.x, ty - self.y
        dist = math.hypot(dx, dy)
        if dist < self.speed:
            self.x, self.y = float(tx), float(ty)
            self.path.pop(0)
        else:
            self.x += (dx / dist) * self.speed
            self.y += (dy / dist) * self.speed
    def draw(self, surface):
        center = (self.x * TILE_SIZE + TILE_SIZE / 2, self.y * TILE_SIZE + TILE_SIZE / 2)
        pygame.draw.circle(surface, self.color, center, TILE_SIZE // 3, 3)

class ExpandingSquarePattern:
    def __init__(self, start_pos):
        self.start_pos = start_pos
    def get_path(self):
        p, lp, l, D = [self.start_pos], self.start_pos, 1, 0
        m = [(0, -1), (1, 0), (0, 1), (-1, 0)]
        while len(p) < 400:
            for _ in range(2):
                for _ in range(l):
                    lp = (lp[0] + m[D][0], lp[1] + m[D][1])
                    if not (0 <= lp[0] < GRID_WIDTH_CELLS and 0 <= lp[1] < GRID_HEIGHT_CELLS): return p
                    p.append(lp)
                D = (D + 1) % 4
            l += 1
        return p

class SectorSearchPattern:
    def __init__(self, start_pos, radius=15, num_sectors=12):
        self.start_pos, self.radius, self.num_sectors = start_pos, radius, num_sectors
    def get_path(self):
        p, s = [self.start_pos], 360 / self.num_sectors
        for i in range(self.num_sectors):
            a = math.radians(i * s)
            ex = round(max(0, min(GRID_WIDTH_CELLS - 1, self.start_pos[0] + self.radius * math.cos(a))))
            ey = round(max(0, min(GRID_HEIGHT_CELLS - 1, self.start_pos[1] - self.radius * math.sin(a))))
            p.append((ex, ey))
            p.append(self.start_pos)
        return p

class ParallelSearchPattern:
    def __init__(self, start_pos):
        self.start_pos = start_pos
    def get_path(self):
        path = [(x, y) for y in range(GRID_HEIGHT_CELLS) for x in (range(GRID_WIDTH_CELLS) if y % 2 == 0 else reversed(range(GRID_WIDTH_CELLS)))]
        path.sort(key=lambda p: abs(p[0] - self.start_pos[0]) + abs(p[1] - self.start_pos[1]))
        return path

class Game:
    _instance = None
    @staticmethod
    def get_instance():
        return Game._instance

    def __init__(self, **kwargs):
        Game._instance = self
        self.mission_settings = kwargs
        self.game_mode = self.mission_settings.get('game_mode', 'ground')
        pygame.init()
        self.screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
        pygame.display.set_caption(f"SAR - {self.game_mode.replace('_', ' ').title()}")
        self.clock = pygame.time.Clock()
        self.font = pygame.font.SysFont("Arial", 20)
        self.running = True
        self.setup_new_mission()

    def is_dead_end(self, x, y):
        targets = getattr(self, 'collectibles', []) + getattr(self, 'land_collectibles', [])
        if (x, y) in targets: return False
        return sum(1 for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)] if 0 <= x + dx < GRID_WIDTH_CELLS and 0 <= y + dy < GRID_HEIGHT_CELLS and self.game_map[y + dy][x + dx] != TILE_TYPE_OBSTACLE) <= 1

    def setup_new_mission(self):
        self.mission_complete, self.mission_failed, self.mission_partial = False, False, False
        self.start_time, self.final_time = pygame.time.get_ticks(), 0
        setups = {"ground": self.setup_ground, "single_drone": self.setup_single_drone, "single_aquatic": self.setup_single_aquatic, "coop_ground_drone": self.setup_coop_ground_drone, "drone_aquatic": self.setup_drone_aquatic, "all_agents": self.setup_all_agents}
        setups.get(self.game_mode, self.setup_ground)()

    def setup_ground(self):
        self.last_positions = collections.deque(maxlen=8); self.loop_counter = 0; self.target_cooldown = {}; self.target_attempts = {}
        self.current_target = None; self.collectibles = []; self.dynamic_obstacles = []
        self.game_map = [[random.choice([0, 0, 0, 0, 1, 1, 2, 3]) for _ in range(GRID_WIDTH_CELLS)] for _ in range(GRID_HEIGHT_CELLS)]
        self.agent = Agent(1, 1)
        self.num_patrol = self.mission_settings.get("num_patrol", random.randint(NUM_PATROL_MIN, NUM_PATROL_MAX))
        self.num_aggressive = self.mission_settings.get("num_aggressive", random.randint(NUM_AGGRESSIVE_MIN, NUM_AGGRESSIVE_MAX))
        for _ in range(self.num_patrol): self.add_obstacle(PatrolObstacle)
        for _ in range(self.num_aggressive): self.add_obstacle(AggressiveObstacle)
        while len(self.collectibles) < 10:
            x, y = random.randint(1, GRID_WIDTH_CELLS - 2), random.randint(1, GRID_HEIGHT_CELLS - 2)
            if self.game_map[y][x] == 0 and (x, y) != (1, 1) and not any((x, y) == (o.x, o.y) for o in self.dynamic_obstacles): self.collectibles.append((x, y))
        self.total_collectibles = len(self.collectibles)

    def setup_single_vehicle(self, is_aquatic):
        self.game_map = [[TILE_TYPE_WATER if is_aquatic else TILE_TYPE_FREE for _ in range(GRID_WIDTH_CELLS)] for _ in range(GRID_HEIGHT_CELLS)]
        for _ in range(70): self.game_map[random.randint(0, GRID_HEIGHT_CELLS - 1)][random.randint(0, GRID_WIDTH_CELLS - 1)] = TILE_TYPE_OBSTACLE
        start_pos = (GRID_WIDTH_CELLS // 2, GRID_HEIGHT_CELLS // 2)
        self.vehicle = Drone(start_pos[0], start_pos[1], color=COLOR_AQUATIC_AGENT if is_aquatic else COLOR_AGENT)
        while True:
            self.target = (random.randint(1, GRID_WIDTH_CELLS - 2), random.randint(1, GRID_HEIGHT_CELLS - 2))
            if self.game_map[self.target[1]][self.target[0]] != TILE_TYPE_OBSTACLE and math.hypot(start_pos[0] - self.target[0], start_pos[1] - self.target[1]) > 5: break
        self.revealed_map = [[False for _ in range(GRID_WIDTH_CELLS)] for _ in range(GRID_HEIGHT_CELLS)]
        self.phase = 'search'
        self.target_found_message = ""
        p_str = self.mission_settings.get('pattern', 'parallel_search'); patterns = {"expanding_square": ExpandingSquarePattern, "sector_search": SectorSearchPattern, "parallel_search": ParallelSearchPattern}
        self.vehicle.set_path(patterns[p_str]((self.vehicle.x, self.vehicle.y)).get_path())

    def setup_single_drone(self): self.setup_single_vehicle(is_aquatic=False)
    def setup_single_aquatic(self): self.setup_single_vehicle(is_aquatic=True)

    def setup_coop_ground_drone(self):
        self.phase="drone_scan"; self.agent=Agent(1,1); self.current_target=None; self.last_positions=collections.deque(maxlen=8); self.loop_counter=0; self.target_cooldown={}; self.target_attempts={}
        self.game_map=[[random.choice([0,0,0,1,2]) for _ in range(GRID_WIDTH_CELLS)] for _ in range(GRID_HEIGHT_CELLS)]; self.collectibles=[]
        while len(self.collectibles)<7:
            x,y=random.randint(1,GRID_WIDTH_CELLS-2),random.randint(1,GRID_HEIGHT_CELLS-2)
            if self.game_map[y][x]==0:self.collectibles.append((x,y))
        self.total_collectibles = len(self.collectibles)
        start_pos=(GRID_WIDTH_CELLS//2,GRID_HEIGHT_CELLS//2); self.drone=Drone(start_pos[0],start_pos[1])
        p_str=self.mission_settings.get('pattern','parallel_search'); patterns={"expanding_square":ExpandingSquarePattern,"sector_search":SectorSearchPattern,"parallel_search":ParallelSearchPattern}
        self.drone.set_path(patterns[p_str]((self.drone.x,self.drone.y)).get_path()); self.revealed_map=[[False]*GRID_WIDTH_CELLS for _ in range(GRID_HEIGHT_CELLS)]

    def setup_drone_aquatic(self):
        self.phase="drone_scan"; self.game_map=[[TILE_TYPE_WATER]*GRID_WIDTH_CELLS for _ in range(GRID_HEIGHT_CELLS)]
        for _ in range(70):self.game_map[random.randint(0,GRID_HEIGHT_CELLS-1)][random.randint(0,GRID_WIDTH_CELLS-1)]=TILE_TYPE_OBSTACLE
        self.drone=Drone(GRID_WIDTH_CELLS//2,2); self.aquatic=Drone(GRID_WIDTH_CELLS//2,GRID_HEIGHT_CELLS-2,color=COLOR_AQUATIC_AGENT)
        while True:
            self.target=(random.randint(1,GRID_WIDTH_CELLS-2),random.randint(1,GRID_HEIGHT_CELLS-2))
            if self.game_map[self.target[1]][self.target[0]]!=1: break
        patterns={"expanding_square":ExpandingSquarePattern,"sector_search":SectorSearchPattern,"parallel_search":ParallelSearchPattern}
        self.drone.set_path(patterns[self.mission_settings.get('drone_pattern')]((self.drone.x,self.drone.y)).get_path())
        self.aquatic.set_path(patterns[self.mission_settings.get('aquatic_pattern')]((self.aquatic.x,self.aquatic.y)).get_path())
        self.revealed_map=[[False]*GRID_WIDTH_CELLS for _ in range(GRID_HEIGHT_CELLS)]

    def setup_all_agents(self):
        self.phase="drone_scan"; self.agent=Agent(1,1); self.current_target=None; self.last_positions=collections.deque(maxlen=8); self.loop_counter=0; self.target_cooldown={}; self.target_attempts={}
        lake_level=GRID_HEIGHT_CELLS//2; self.game_map=[[TILE_TYPE_WATER if y>lake_level else TILE_TYPE_FREE for x in range(GRID_WIDTH_CELLS)] for y in range(GRID_HEIGHT_CELLS)]
        for _ in range(80):self.game_map[random.randint(0,GRID_HEIGHT_CELLS-1)][random.randint(0,GRID_WIDTH_CELLS-1)]=TILE_TYPE_OBSTACLE
        self.drone=Drone(GRID_WIDTH_CELLS//2,2); self.aquatic=Drone(GRID_WIDTH_CELLS//2,GRID_HEIGHT_CELLS-2,color=COLOR_AQUATIC_AGENT)
        self.land_collectibles=[]; self.water_collectibles=[]
        while len(self.land_collectibles)<3:
            x,y=random.randint(1,GRID_WIDTH_CELLS-2),random.randint(1,lake_level)
            if self.game_map[y][x]==0: self.land_collectibles.append((x,y))
        while len(self.water_collectibles)<2:
            x,y=random.randint(1,GRID_WIDTH_CELLS-2),random.randint(lake_level+1,GRID_HEIGHT_CELLS-2)
            if self.game_map[y][x]==4: self.water_collectibles.append((x,y))
        self.total_collectibles = len(self.land_collectibles) + len(self.water_collectibles)
        p_str=self.mission_settings.get('pattern','parallel_search'); patterns={"expanding_square":ExpandingSquarePattern,"sector_search":SectorSearchPattern,"parallel_search":ParallelSearchPattern}
        self.drone.set_path(patterns[p_str]((self.drone.x,self.drone.y)).get_path()); self.aquatic.set_path(ParallelSearchPattern((self.aquatic.x,self.aquatic.y)).get_path())
        self.revealed_map=[[False]*GRID_WIDTH_CELLS for _ in range(GRID_HEIGHT_CELLS)]

    def add_obstacle(self, obs_type):
        for _ in range(100):
            x, y = random.randint(1, GRID_WIDTH_CELLS - 2), random.randint(1, GRID_HEIGHT_CELLS - 2)
            if self.game_map[y][x] == 0 and math.hypot(x - 1, y - 1) > 6 and not any(math.hypot(x - o.x, y - o.y) <= 2 for o in self.dynamic_obstacles):
                self.dynamic_obstacles.append(obs_type(x, y, self.game_map, self.agent) if obs_type == AggressiveObstacle else obs_type(x, y, self.game_map)); return

    def run(self):
        self.back_to_menu = False
        updates = {"ground": self.update_ground, "single_drone": self.update_single_vehicle, "single_aquatic": self.update_single_vehicle, "coop_ground_drone": self.update_coop_ground_drone, "drone_aquatic": self.update_drone_aquatic, "all_agents": self.update_all_agents}
        self.find_path()
        while self.running:
            self.handle_events()
            if not any([self.mission_complete, self.mission_failed, self.mission_partial]):
                updates.get(self.game_mode, self.update_ground)()
            self.draw(); self.clock.tick(7)
            if any([self.mission_complete, self.mission_failed, self.mission_partial]):
                self.handle_end_of_mission_events()
        return self.back_to_menu

    def handle_events(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT: self.running = False; self.back_to_menu = False
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE: self.running = False; self.back_to_menu = True
                if event.key == pygame.K_r and not any([self.mission_complete, self.mission_failed, self.mission_partial]):
                    self.setup_new_mission()
                    self.find_path()

    def handle_end_of_mission_events(self):
        waiting = True
        while waiting and self.running:
            self.draw()
            for event in pygame.event.get():
                if event.type == pygame.QUIT: self.running = False; waiting = False
                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_r: self.setup_new_mission(); self.find_path(); waiting = False
                    if event.key == pygame.K_ESCAPE: self.running = False; self.back_to_menu = True; waiting = False
            self.clock.tick(10)

    def update_ground(self):
        pos=(self.agent.x,self.agent.y)
        if self.current_target and self.target_attempts.get(self.current_target,0)>10:
            self.target_cooldown[self.current_target]=25; self.current_target=None; self.agent.path=[]
        dynamic_obstacles=getattr(self,'dynamic_obstacles',[])
        for obs in dynamic_obstacles:
            obs.update([TILE_TYPE_FREE,TILE_TYPE_DIFFICULT,TILE_TYPE_HAZARD])
            if(obs.x,obs.y)==pos: self.end_mission("failed"); return
        threats=[o for o in dynamic_obstacles if self.turns_to_reach((o.x,o.y),pos)<=(o.detection_radius if isinstance(o,AggressiveObstacle)else 2)]
        if threats:
            brave_path = self.find_brave_path(threats)
            if brave_path:
                self.current_target = brave_path[-1]; self.agent.set_path(brave_path)
            else:
                escape_path = self.find_escape_path(threats, dynamic_obstacles)
                if escape_path:
                    self.current_target = None; self.agent.set_path(escape_path)
                else:
                    best_move, max_dist = None, -1; closest_threat = min(threats, key=lambda t:math.hypot(pos[0]-t.x, pos[1]-t.y))
                    for dx,dy in [(x,y) for x in [-1,0,1] for y in [-1,0,1] if not(x==0 and y==0)]:
                        nx, ny = pos[0]+dx, pos[1]+dy
                        if 0<=nx<GRID_WIDTH_CELLS and 0<=ny<GRID_HEIGHT_CELLS and self.game_map[ny][nx] not in [1,4] and not any((nx,ny)==(o.x,o.y) for o in dynamic_obstacles):
                            dist = math.hypot(nx-closest_threat.x, ny-closest_threat.y)
                            if dist > max_dist: max_dist = dist; best_move = (nx, ny)
                    if best_move: self.agent.set_path([best_move])
                    else: self.agent.path = []; self.agent.wait_timer = 1
        elif not self.agent.path or self.current_target not in self.collectibles:
            self.find_path_to_collectible(self.collectibles, [TILE_TYPE_WATER,TILE_TYPE_OBSTACLE])
        self.agent.move()
        if(self.agent.x,self.agent.y)in self.collectibles:
            self.collectibles.remove((self.agent.x,self.agent.y)); self.agent.score+=10
            if(self.agent.x,self.agent.y)in self.target_attempts: del self.target_attempts[(self.agent.x,self.agent.y)]
            self.current_target=None; self.agent.path=[]
            if not self.collectibles: self.end_mission("success")

    def update_single_vehicle(self):
        if not self.vehicle.path:
            self.end_mission("failed"); return
        if self.phase == 'search':
            self.vehicle.move()
            vx, vy = int(self.vehicle.x), int(self.vehicle.y)
            radius = int(self.vehicle.vision_radius)
            for y in range(max(0, vy - radius), min(GRID_HEIGHT_CELLS, vy + radius + 1)):
                for x in range(max(0, vx - radius), min(GRID_WIDTH_CELLS, vx + radius + 1)):
                    if math.hypot(x - self.vehicle.x, y - self.vehicle.y) <= self.vehicle.vision_radius:
                        self.revealed_map[y][x] = True
            if self.revealed_map[self.target[1]][self.target[0]]:
                self.phase = 'goto_target'
                self.target_found_message = f"Target spotted at: {self.target}"
                start_node = (int(self.vehicle.x), int(self.vehicle.y))
                path, _ = self.a_star(start_node, self.target, [TILE_TYPE_OBSTACLE])
                if path: self.vehicle.set_path(path)
                else: self.end_mission("failed")
        elif self.phase == 'goto_target':
            self.vehicle.move()
            if math.hypot(self.vehicle.x - self.target[0], self.vehicle.y - self.target[1]) < 1.0:
                self.end_mission("success")

    def drone_scan_phase(self, drone, revealed_map):
        if drone.path:
            drone.move();
            for y in range(GRID_HEIGHT_CELLS):
                for x in range(GRID_WIDTH_CELLS):
                    if not revealed_map[y][x] and math.hypot(x - drone.x, y - drone.y) <= drone.vision_radius: revealed_map[y][x] = True
            return False
        return True

    def update_coop_ground_drone(self):
        if self.phase=="drone_scan":
            if self.drone_scan_phase(self.drone,self.revealed_map): self.phase="ground_search"
        elif self.phase=="ground_search":
            if not self.collectibles: self.end_mission("success"); return
            self.update_ground()

    def update_drone_aquatic(self):
        if self.phase=="drone_scan":
            if self.drone_scan_phase(self.drone,self.revealed_map): self.phase="aquatic_search"
        elif self.phase=="aquatic_search":
            if self.aquatic.path:
                self.aquatic.move()
                if math.hypot(self.aquatic.x-self.target[0],self.aquatic.y-self.target[1])<=1.0: self.end_mission("success")
            else: self.end_mission("failed")

    def update_all_agents(self):
        if self.phase=="drone_scan":
            if self.drone_scan_phase(self.drone,self.revealed_map): self.phase="coordinated_search"
        elif self.phase=="coordinated_search":
            if self.land_collectibles:
                self.collectibles=self.land_collectibles; self.dynamic_obstacles=[]; self.update_ground()
                self.land_collectibles=self.collectibles
            if self.water_collectibles:
                self.aquatic.move()
                collected=next((t for t in self.water_collectibles if math.hypot(self.aquatic.x-t[0],self.aquatic.y-t[1])<1.5),None)
                if collected: self.water_collectibles.remove(collected)
            if not self.land_collectibles and not self.water_collectibles: self.end_mission("success")

    def find_path(self):
        if hasattr(self, 'agent') and self.game_mode == "ground":
            self.find_path_to_collectible(self.collectibles, [TILE_TYPE_WATER, TILE_TYPE_OBSTACLE])

    def find_path_to_collectible(self,targets,restricted_tiles):
        if not targets: return
        pos=(self.agent.x,self.agent.y); cost_map=self.create_dynamic_cost_map()
        available=[t for t in targets if t not in self.target_cooldown]
        best_path,best_cost,best_target=None,float('inf'),None
        for target in available:
            path,cost = self.a_star(pos,target,restricted_tiles,cost_map)
            if path and cost<best_cost: best_path,best_cost,best_target=path,cost,target
        if best_path:
            self.current_target=best_target; self.agent.set_path(best_path)
            self.target_attempts[best_target]=self.target_attempts.get(best_target,0)+1
        else: self.agent.path=[]; self.current_target=None

    def find_escape_path(self,threats,dynamic_obstacles):
        pos,cost_map=(self.agent.x,self.agent.y),self.create_dynamic_cost_map(True)
        best_path,best_score=None,-float('inf')
        candidates=[(pos[0]+i,pos[1]+j) for r in range(2,8) for i in range(-r,r+1) for j in range(-r,r+1) if abs(i)+abs(j)==r and 0<=pos[0]+i<GRID_WIDTH_CELLS and 0<=pos[1]+j<GRID_HEIGHT_CELLS and self.game_map[pos[1]+j][pos[0]+i] not in [1,4]]
        for spot in candidates:
            if any(spot==(o.x,o.y) for o in dynamic_obstacles): continue
            path,cost=self.a_star(pos,spot,[TILE_TYPE_WATER,TILE_TYPE_OBSTACLE],cost_map)
            if path and not any(s==(o.x,o.y) for s in path for o in dynamic_obstacles):
                score=min(self.turns_to_reach((t.x,t.y),spot) for t in threats)*1.5-len(path)
                if score>best_score: best_path,best_score=path,score
        return best_path

    def find_brave_path(self, threats):
        agent_pos = (self.agent.x, self.agent.y)
        aggressive_threats = [t for t in threats if isinstance(t, AggressiveObstacle)]
        if not aggressive_threats: return None
        sorted_collectibles = sorted(self.collectibles, key=lambda c: abs(c[0]-agent_pos[0]) + abs(c[1]-agent_pos[1]))
        for target in sorted_collectibles:
            if self.is_dead_end(target[0], target[1]): continue
            agent_steps = self.turns_to_reach(agent_pos, target)
            if agent_steps == float('inf'): continue
            enemy_steps = min(self.turns_to_reach((t.x, t.y), target) for t in aggressive_threats)
            if agent_steps < enemy_steps - 1:
                cost_map = self.create_dynamic_cost_map(escape=True)
                path, cost = self.a_star(agent_pos, target, [TILE_TYPE_WATER, TILE_TYPE_OBSTACLE], cost_map)
                if path: return path
        return None

    def create_dynamic_cost_map(self,escape=False):
        cost_map=[[1+self.game_map[y][x]*5 for x in range(GRID_WIDTH_CELLS)] for y in range(GRID_HEIGHT_CELLS)]
        for y in range(GRID_HEIGHT_CELLS):
            for x in range(GRID_WIDTH_CELLS):
                if self.is_dead_end(x,y):
                    penalty = 4000 if escape else 200; cost_map[y][x]+=penalty
        for obs in getattr(self,'dynamic_obstacles',[]):
            if isinstance(obs,AggressiveObstacle):
                r,bc,ac=(obs.detection_radius,5000,1500)
                for dy in range(-r,r+1):
                    for dx in range(-r,r+1):
                        nx,ny=obs.x+dx,obs.y+dy
                        if 0<=nx<GRID_WIDTH_CELLS and 0<=ny<GRID_HEIGHT_CELLS:
                            dist=abs(dx)+abs(dy)
                            if dist==0:cost_map[ny][nx]+=bc
                            elif dist==1:cost_map[ny][nx]+=ac
            else:
                cost_map[obs.y][obs.x]+=2500
                for dx,dy in [(0,1),(0,-1),(1,0),(-1,0)]:
                    nx,ny=obs.x+dx,obs.y+dy
                    if 0<=nx<GRID_WIDTH_CELLS and 0<=ny<GRID_HEIGHT_CELLS and self.game_map[ny][nx] not in [1,4]:
                        cost_map[ny][nx]+=1200
        return cost_map

    def turns_to_reach(self,start,end):
        path, cost = self.a_star(start,end,[1,4]); return cost

    def a_star(self,start,end,restricted,cost_map=None):
        if cost_map is None: cost_map = [[1]*GRID_WIDTH_CELLS for _ in range(GRID_HEIGHT_CELLS)]
        nodes,closed,g_costs=[(0,Node(start))],set(),{start:0}
        while nodes:
            _,c_node=heapq.heappop(nodes)
            if c_node.position==end:
                path=[];p=c_node
                while p:path.append(p.position);p=p.parent
                return path[::-1],g_costs.get(end, float('inf'))
            if c_node.position in closed:continue
            closed.add(c_node.position)
            for move in [(0,-1),(0,1),(-1,0),(1,0)]:
                pos=(c_node.position[0]+move[0],c_node.position[1]+move[1])
                if not(0<=pos[0]<GRID_WIDTH_CELLS and 0<=pos[1]<GRID_HEIGHT_CELLS)or self.game_map[pos[1]][pos[0]] in restricted or pos in closed:continue
                new_g=g_costs.get(c_node.position,float('inf'))+cost_map[pos[1]][pos[0]]
                if new_g<g_costs.get(pos,float('inf')):
                    g_costs[pos]=new_g;h=abs(pos[0]-end[0])+abs(pos[1]-end[1])
                    heapq.heappush(nodes,(new_g+h,Node(pos,c_node)))
        return None, float('inf')

    def end_mission(self,result):
        if self.mission_complete or self.mission_failed or self.mission_partial:return
        self.final_time=(pygame.time.get_ticks()-self.start_time)//1000
        if result == "failed" and hasattr(self, 'total_collectibles') and self.total_collectibles > 0:
            collectibles_left = len(getattr(self, 'collectibles', []))
            if self.game_mode == 'all_agents': collectibles_left = len(self.land_collectibles) + len(self.water_collectibles)
            collected = self.total_collectibles - collectibles_left
            if (self.total_collectibles > 0 and (collected / self.total_collectibles) > 0.7):
                result = "partial"
        if result=="success":self.mission_complete=True
        elif result=="partial":self.mission_partial=True
        else:self.mission_failed=True

    def draw(self):
        self.screen.fill(BLACK); self.draw_grid()
        draw_funcs={"ground":self.draw_ground,"single_drone":self.draw_single_vehicle,"single_aquatic":self.draw_single_vehicle,"coop_ground_drone":self.draw_coop_ground_drone,"drone_aquatic":self.draw_drone_aquatic,"all_agents":self.draw_all_agents}
        draw_funcs.get(self.game_mode,self.draw_ground)(); self.draw_ui(); pygame.display.flip()

    def draw_grid(self):
        is_fog_of_war_mode = hasattr(self, 'revealed_map')
        for y in range(GRID_HEIGHT_CELLS):
            for x in range(GRID_WIDTH_CELLS):
                if is_fog_of_war_mode and not self.revealed_map[y][x]:
                    pygame.draw.rect(self.screen, (20, 20, 20), (x * TILE_SIZE, y * TILE_SIZE, TILE_SIZE, TILE_SIZE))
                    continue
                tile = self.game_map[y][x]
                color = COLOR_OBSTACLE if tile == 1 else COLOR_DIFFICULT if tile == 2 else COLOR_HAZARD if tile == 3 else COLOR_WATER if tile == 4 else COLOR_FREE
                pygame.draw.rect(self.screen, color, (x * TILE_SIZE, y * TILE_SIZE, TILE_SIZE, TILE_SIZE))
                if color != COLOR_WATER: pygame.draw.rect(self.screen, BLACK, (x * TILE_SIZE, y * TILE_SIZE, TILE_SIZE, TILE_SIZE), 1)

    def draw_ground(self):
        if hasattr(self, 'agent') and self.agent.path: self.draw_path(self.agent.path)
        if hasattr(self, 'collectibles'):
            for c in self.collectibles: self.draw_collectible(c)
        if hasattr(self, 'current_target') and self.current_target: self.draw_goal_outline(self.current_target)
        if hasattr(self, 'dynamic_obstacles'):
            for obs in self.dynamic_obstacles: obs.draw(self.screen)
        if hasattr(self, 'agent'): self.agent.draw(self.screen)

    def draw_single_vehicle(self):
        self.draw_collectible(self.target); self.vehicle.draw(self.screen)

    def draw_coop_ground_drone(self):
        if self.phase == "ground_search" and self.agent.path: self.draw_path(self.agent.path)
        for c in self.collectibles: self.draw_collectible(c)
        if self.phase == "drone_scan": self.drone.draw(self.screen)
        else: self.agent.draw(self.screen)

    def draw_drone_aquatic(self):
        self.draw_collectible(self.target); self.drone.draw(self.screen); self.aquatic.draw(self.screen)

    def draw_all_agents(self):
        if self.phase=="coordinated_search" and self.agent.path: self.draw_path(self.agent.path)
        for c in self.land_collectibles+self.water_collectibles: self.draw_collectible(c)
        self.agent.draw(self.screen); self.aquatic.draw(self.screen)
        if self.phase=="drone_scan": self.drone.draw(self.screen)

    def draw_path(self,path):
        path_surf=pygame.Surface((SCREEN_WIDTH,SCREEN_HEIGHT),pygame.SRCALPHA)
        for i,step in enumerate(path):
            alpha=max(50,200-i*15)
            pygame.draw.rect(path_surf,(*COLOR_PATH[:3],alpha),(step[0]*TILE_SIZE+5,step[1]*TILE_SIZE+5,TILE_SIZE-10,TILE_SIZE-10),border_radius=3)
        self.screen.blit(path_surf,(0,0))

    def draw_collectible(self,pos):
        pygame.draw.circle(self.screen,COLOR_COLLECTIBLE,(pos[0]*TILE_SIZE+TILE_SIZE//2,pos[1]*TILE_SIZE+TILE_SIZE//2),TILE_SIZE//3)

    def draw_goal_outline(self,pos):
        pygame.draw.rect(self.screen,COLOR_GOAL_OUTLINE,(pos[0]*TILE_SIZE,pos[1]*TILE_SIZE,TILE_SIZE,TILE_SIZE),3)

    def draw_ui(self):
        time_val=(pygame.time.get_ticks()-self.start_time)//1000 if not self.final_time else self.final_time
        time_text=self.font.render(f"Time: {time_val}",True,WHITE); self.screen.blit(time_text,(10,10))
        if hasattr(self,'total_collectibles') and self.total_collectibles > 0:
            current_collectibles = len(getattr(self,'collectibles', []))
            if self.game_mode == 'all_agents': current_collectibles = len(self.land_collectibles) + len(self.water_collectibles)
            collected = self.total_collectibles - current_collectibles
            targets_text = self.font.render(f"Targets: {collected} / {self.total_collectibles}", True, WHITE)
            self.screen.blit(targets_text, (150, 10))
        if self.game_mode == "ground":
            enemies_text = self.font.render(f"Enemies: {self.num_patrol + self.num_aggressive} ({self.num_patrol}P, {self.num_aggressive}A)", True, WHITE)
            self.screen.blit(enemies_text, (SCREEN_WIDTH - 250, 10))
        if hasattr(self, 'target_found_message') and self.target_found_message:
            msg_surf = self.font.render(self.target_found_message, True, (0, 255, 0))
            rect = msg_surf.get_rect(center=(SCREEN_WIDTH / 2, SCREEN_HEIGHT - 20))
            self.screen.blit(msg_surf, rect)
        msg,color=("",(0,0,0))
        if self.mission_complete: msg,color="APOSTOLI EPITYCHIS! (R for New)",(0,255,0)
        elif self.mission_partial: msg,color="MERIKH EPITYCHIA! (R for New)",(255,255,0)
        elif self.mission_failed: msg,color="APOSTOLI APETYXE! (R for New)",(255,0,0)
        if msg:
            msg_surf=self.font.render(msg,True,color); rect=msg_surf.get_rect(center=(SCREEN_WIDTH/2,SCREEN_HEIGHT/2))
            pygame.draw.rect(self.screen,BLACK,rect.inflate(20,10)); self.screen.blit(msg_surf,rect)

def main_menu():
    screen = pygame.display.set_mode((550, 400))
    pygame.display.set_caption("SAR - Κεντρικό Μενού")
    font = pygame.font.SysFont("Arial", 24)
    selected = 0
    options = ["1. Επίγειο Ρομπότ", "2. Αποστολή Drone", "3. Αποστολή Υδάτινου", "4. Drone + Επίγειο (Cooperative)", "5. Drone + Υδάτινο", "6. Όλα Μαζί (Λίμνη)"]
    keys = ["ground", "single_drone", "single_aquatic", "coop_ground_drone", "drone_aquatic", "all_agents"]
    while True:
        screen.fill(MENU_BG_COLOR)
        title = font.render("SAR - Επιλογή Σεναρίου", True, (255, 255, 0))
        screen.blit(title, title.get_rect(center=(275, 40)))
        for i, opt in enumerate(options):
            color = MENU_HIGHLIGHT_COLOR if i == selected else MENU_TEXT_COLOR
            screen.blit(font.render(opt, True, color), font.render(opt, True, color).get_rect(center=(275, 100 + i * 45)))
        pygame.display.flip()
        for event in pygame.event.get():
            if event.type == pygame.QUIT: return None
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_DOWN: selected = (selected + 1) % len(options)
                if event.key == pygame.K_UP: selected = (selected - 1) % len(options)
                if event.key == pygame.K_RETURN: return keys[selected]
                if event.key == pygame.K_ESCAPE: return None

def pattern_menu(title):
    screen = pygame.display.set_mode((550, 300))
    pygame.display.set_caption(title)
    font = pygame.font.SysFont("Arial", 24)
    selected = 0
    patterns = ["expanding_square", "sector_search", "parallel_search"]
    labels = ["Expanding Square", "Sector Search", "Parallel Track"]
    while True:
        screen.fill(MENU_BG_COLOR)
        title_surf = font.render(title, True, (255, 255, 0))
        screen.blit(title_surf, title_surf.get_rect(center=(275, 40)))
        for i, label in enumerate(labels):
            color = MENU_HIGHLIGHT_COLOR if i == selected else MENU_TEXT_COLOR
            screen.blit(font.render(label, True, color), font.render(label, True, color).get_rect(center=(275, 100 + i * 40)))
        pygame.display.flip()
        for event in pygame.event.get():
            if event.type == pygame.QUIT: return "quit"
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_DOWN: selected = (selected + 1) % len(labels)
                if event.key == pygame.K_UP: selected = (selected - 1) % len(labels)
                if event.key == pygame.K_RETURN: return patterns[selected]
                if event.key == pygame.K_ESCAPE: return None

def enemy_selection_menu():
    screen = pygame.display.set_mode((550, 300))
    pygame.display.set_caption("SAR - Ρύθμιση Εχθρών")
    font = pygame.font.SysFont("Arial", 24)
    clock = pygame.time.Clock()
    num_patrol, num_aggressive = 3, 2
    min_val, max_patrol, max_aggr = 0, 5, 3
    selected_option = 0
    while True:
        screen.fill(MENU_BG_COLOR)
        title = font.render("Ρύθμιση Εχθρών", True, (255, 255, 0))
        screen.blit(title, title.get_rect(center=(275, 40)))
        color_p = MENU_HIGHLIGHT_COLOR if selected_option == 0 else MENU_TEXT_COLOR
        patrol_txt = font.render(f"Εχθροί Περιπολίας: < {num_patrol} >", True, color_p)
        screen.blit(patrol_txt, patrol_txt.get_rect(center=(275, 120)))
        color_a = MENU_HIGHLIGHT_COLOR if selected_option == 1 else MENU_TEXT_COLOR
        aggr_txt = font.render(f"Επιθετικοί Εχθροί: < {num_aggressive} >", True, color_a)
        screen.blit(aggr_txt, aggr_txt.get_rect(center=(275, 170)))
        instr = font.render("Enter: Έναρξη, Esc: Πίσω", True, (200, 200, 200))
        screen.blit(instr, instr.get_rect(center=(275, 250)))
        pygame.display.flip()
        for event in pygame.event.get():
            if event.type == pygame.QUIT: return None
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE: return None
                if event.key == pygame.K_RETURN: return num_patrol, num_aggressive
                if event.key in [pygame.K_UP, pygame.K_DOWN]: selected_option = 1 - selected_option
                if event.key == pygame.K_LEFT:
                    if selected_option == 0: num_patrol = max(min_val, num_patrol - 1)
                    else: num_aggressive = max(min_val, num_aggressive - 1)
                if event.key == pygame.K_RIGHT:
                    if selected_option == 0: num_patrol = min(max_patrol, num_patrol + 1)
                    else: num_aggressive = min(max_aggr, num_aggressive + 1)
        clock.tick(30)

def main():
    pygame.init()
    while True:
        game_mode = main_menu()
        if game_mode is None: break
        params = {"game_mode": game_mode}
        if game_mode == "ground":
            enemy_settings = enemy_selection_menu()
            if enemy_settings is None: continue
            params["num_patrol"], params["num_aggressive"] = enemy_settings
        if game_mode in ["single_drone", "coop_ground_drone", "all_agents"]:
            p = pattern_menu("Επιλογή Μοτίβου Drone")
            if p in [None, "quit"]: continue
            params["pattern"] = p
        elif game_mode == "single_aquatic":
            p = pattern_menu("Επιλογή Μοτίβου Υδάτινου")
            if p in [None, "quit"]: continue
            params["pattern"] = p
        elif game_mode == "drone_aquatic":
            dp = pattern_menu("Επιλογή Μοτίβου Drone")
            if dp in [None, "quit"]: continue
            params["drone_pattern"]=dp
            ap = pattern_menu("Επιλογή Μοτίβου Υδάτινου")
            if ap in [None, "quit"]: continue
            params["aquatic_pattern"] = ap

        game = Game(**params)
        if not game.run(): break
    pygame.quit()

if __name__ == '__main__':
    main()