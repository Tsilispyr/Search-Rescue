# -*- coding: utf-8 -*-
"""
SAR - Search and Rescue Simulation System
Σύστημα προσομοίωσης για διαχείριση αυτόνομων οχημάτων σε αποστολές έρευνας και διάσωσης

Αυτό το σύστημα περιλαμβάνει:
- Διάφορους τύπους οχημάτων (επίγεια ρομπότ, drones, υδάτινα οχήματα)
- Αλγορίθμους pathfinding (A*)
- Μοτίβα αναζήτησης (expanding square, sector search, parallel track)
- Δυναμική αποφυγή εχθρών
- Fog of war για drones
- Πολλαπλά σενάρια αποστολών

Δημιουργήθηκε για διπλωματική εργασία στο Χαροκόπειο Πανεπιστήμιο
"""

import pygame
import random
import math
import heapq
import json
import collections
import os


# ΣΤΑΘΕΡΕΣ ΚΑΙ ΠΑΡΑΜΕΤΡΟΙ

# Διαστάσεις πλέγματος και οθόνης
TILE_SIZE = 30                    # Μέγεθος κάθε κελιού σε pixels
GRID_WIDTH_CELLS = 25             # Πλάτος πλέγματος σε κελιά
GRID_HEIGHT_CELLS = 20            # Ύψος πλέγματος σε κελιά
SCREEN_WIDTH = GRID_WIDTH_CELLS * TILE_SIZE    # Πλάτος οθόνης
SCREEN_HEIGHT = GRID_HEIGHT_CELLS * TILE_SIZE  # Ύψος οθόνης

# Προεπιλεγμένες παράμετροι αποστολής
NUM_PATROL_MIN, NUM_PATROL_MAX = 3, 5          # Εύρος εχθρών περιπολίας
NUM_AGGRESSIVE_MIN, NUM_AGGRESSIVE_MAX = 2, 3  # Εύρος επιθετικών εχθρών


# ΧΡΩΜΑΤΑ ΚΑΙ ΤΥΠΟΙ ΚΕΛΙΩΝ

# Βασικά χρώματα
BLACK, WHITE = (0, 0, 0), (255, 255, 255)

# Χρώματα για διάφορους τύπους εδάφους
COLOR_FREE = (200, 200, 200)          # Ελεύθερο έδαφος
COLOR_OBSTACLE = (50, 50, 50)         # Εμπόδιο
COLOR_DIFFICULT = (100, 100, 160)     # Δύσκολο έδαφος
COLOR_HAZARD = (255, 180, 180)        # Κίνδυνος
COLOR_WATER = (30, 90, 180)           # Νερό

# Χρώματα οχημάτων
COLOR_AGENT = (0, 120, 255)           # Επίγειο ρομπότ
COLOR_AQUATIC_AGENT = (0, 200, 200)   # Υδάτινο όχημα

# Χρώματα UI
COLOR_GOAL_OUTLINE = (255, 215, 0)    # Περίγραμμα στόχου
COLOR_PATH = (0, 150, 0, 150)         # Διαδρομή (με διαφάνεια)
COLOR_COLLECTIBLE = (255, 255, 0)     # Συλλεκτό αντικείμενο

# Χρώματα εχθρών
COLOR_PATROL_OBSTACLE = (180, 0, 0)      # Εχθρός περιπολίας
COLOR_AGGRESSIVE_OBSTACLE = (255, 0, 255) # Επιθετικός εχθρός

# Χρώματα μενού
MENU_BG_COLOR = (10, 25, 47)          # Φόντο μενού
MENU_TEXT_COLOR = (200, 200, 255)     # Κείμενο μενού
MENU_HIGHLIGHT_COLOR = (255, 255, 0)  # Επιλεγμένο στοιχείο

# Τύποι κελιών (αριθμητικοί κωδικοί)
TILE_TYPE_FREE = 0        # Ελεύθερο έδαφος
TILE_TYPE_OBSTACLE = 1    # Εμπόδιο
TILE_TYPE_DIFFICULT = 2   # Δύσκολο έδαφος
TILE_TYPE_HAZARD = 3      # Κίνδυνος
TILE_TYPE_WATER = 4       # Νερό

# ΚΛΑΣΕΙΣ ΓΙΑ PATHFINDING

class Node:
    """ Κλάση κόμβου για τον A* αλγόριθμο pathfinding. Κάθε κόμβος αντιπροσωπεύει ένα σημείο στο πλέγμα. """
    def __init__(self, position, parent=None):
        self.parent = parent          # Γονικός κόμβος για την κατασκευή διαδρομής
        self.position = position      # Θέση (x, y) στο πλέγμα
        self.g = 0                    # Κόστος από την αρχή
        self.h = 0                    # Ευρετική εκτίμηση προς τον στόχο
        self.f = 0                    # Συνολικό κόστος (g + h)
    
    def __eq__(self, other): 
        """Έλεγχος ισότητας δύο κόμβων"""
        return self.position == other.position 
    
    def __lt__(self, other): 
        """Σύγκριση κόμβων για το heap"""
        return self.f < other.f
    
    def __hash__(self): 
        """Hash function για χρήση σε sets"""
        return hash(self.position)


# ΚΛΑΣΕΙΣ ΟΧΗΜΑΤΩΝ

class Agent:
    """ Κλάση για το επίγειο ρομπότ. Κινείται σε ξηρά εδάφη, συλλέγει αντικείμενα και αποφεύγει εχθρούς. """
    def __init__(self, x, y):
        self.x, self.y = x, y         # Τρέχουσα θέση
        self.path = []                # Διαδρομή προς τον στόχο
        self.score = 0                # Βαθμολογία (συλλεγμένα αντικείμενα)
        self.wait_timer = 0           # Χρονόμετρο αναμονής
    
    def set_path(self, path):
        """ Ορίζει νέα διαδρομή για το ρομπότ. Αφαιρεί το πρώτο σημείο αν είναι η τρέχουσα θέση. """
        if path and path[0] == (self.x, self.y):
            self.path = path[1:]      # Αφαίρεση πρώτου σημείου
        elif path:
            self.path = path
        else:
            self.path = []
    
    def move(self):
        """ Κινεί το ρομπότ κατά μία θέση προς τον στόχο. Ελέγχει για αναμονή και εγκυρότητα κίνησης. """
        if self.wait_timer > 0: 
            self.wait_timer -= 1
            return
        
        if self.path and abs(self.path[0][0] - self.x) + abs(self.path[0][1] - self.y) == 1:
            # Κίνηση μόνο σε γειτονικά κελιά
            self.x, self.y = self.path.pop(0)
        else:
            self.path = []  # Ακύρωση διαδρομής αν δεν είναι έγκυρη
    
    def draw(self, surface):
        """Σχεδιάζει το ρομπότ στην οθόνη"""
        rect = pygame.Rect(self.x * TILE_SIZE, self.y * TILE_SIZE, TILE_SIZE, TILE_SIZE)
        pygame.draw.rect(surface, COLOR_AGENT, rect, 0, 4)  # Στρογγυλεμένα γωνίες


# ΚΛΑΣΕΙΣ ΕΧΘΡΩΝ

class PatrolObstacle:
    """ Εχθρός περιπολίας που κινείται τυχαία στο χάρτη. Δεν επιτίθεται απευθείας στον παίκτη. """
    def __init__(self, x, y, game_map):
        self.x, self.y = x, y         # Τρέχουσα θέση
        self.game_map = game_map      # Αναφορά στον χάρτη
    
    def update(self, allowed_tiles):
        """ Ενημερώνει τη θέση του εχθρού με τυχαία κίνηση. allowed_tiles: λίστα επιτρεπόμενων τύπων κελιών """
        # Εύρεση δυνατών κινήσεων (μόνο γειτονικά κελιά)
        moves = []
        for dx, dy in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
            new_x, new_y = self.x + dx, self.y + dy
            if (0 <= new_x < GRID_WIDTH_CELLS and 
                0 <= new_y < GRID_HEIGHT_CELLS and 
                self.game_map[new_y][new_x] in allowed_tiles):
                moves.append((dx, dy))
        
        if moves:
            # Τυχαία επιλογή κίνησης
            dx, dy = random.choice(moves)
            self.x, self.y = self.x + dx, self.y + dy
    
    def draw(self, surface):
        """Σχεδιάζει τον εχθρό περιπολίας"""
        rect = pygame.Rect(self.x * TILE_SIZE, self.y * TILE_SIZE, TILE_SIZE, TILE_SIZE)
        pygame.draw.ellipse(surface, COLOR_PATROL_OBSTACLE, rect)

class AggressiveObstacle(PatrolObstacle):
    """ Επιθετικός εχθρός που αλλάζει συμπεριφορά ανάλογα με την απόσταση από τον παίκτη. Σε λειτουργία 'patrol': κινείται τυχαία Σε λειτουργία 'attack': επιτίθεται στον παίκτη """
    def __init__(self, x, y, game_map, agent):
        super().__init__(x, y, game_map)
        self.agent = agent                    # Αναφορά στον παίκτη
        self.detection_radius = 6             # Ακτίνα εντοπισμού
        self.mode = 'patrol'                  # Τρέχουσα λειτουργία
    
    def update(self, allowed_tiles):
        """ Ενημερώνει τη συμπεριφορά του εχθρού. Αλλάζει σε λειτουργία attack αν ο παίκτης είναι κοντά. """
        # Υπολογισμός απόστασης από τον παίκτη
        dist = math.hypot(self.x - self.agent.x, self.y - self.agent.y)
        
        # Αλλαγή λειτουργίας ανάλογα με την απόσταση
        self.mode = 'attack' if dist <= self.detection_radius else 'patrol'
        
        if self.mode == 'attack':
            # Λειτουργία επιθέσεως: χρήση A* για εύρεση διαδρομής προς τον παίκτη
            game = Game.get_instance()
            if game:
                path_res, _ = game.a_star((self.x, self.y), (self.agent.x, self.agent.y), 
                                        [TILE_TYPE_OBSTACLE, TILE_TYPE_WATER])
                if path_res and len(path_res) > 1:
                    self.x, self.y = path_res[1]  # Κίνηση στο επόμενο σημείο
                else:
                    super().update(allowed_tiles)  # Επιστροφή σε τυχαία κίνηση
        else:
            # Λειτουργία περιπολίας: τυχαία κίνηση
            super().update(allowed_tiles)
    
    def draw(self, surface):
        """Σχεδιάζει τον επιθετικό εχθρό με διαφορετικό χρώμα ανάλογα με τη λειτουργία"""
        rect = pygame.Rect(self.x * TILE_SIZE, self.y * TILE_SIZE, TILE_SIZE, TILE_SIZE)
        color = COLOR_AGGRESSIVE_OBSTACLE if self.mode == 'attack' else COLOR_PATROL_OBSTACLE
        pygame.draw.ellipse(surface, color, rect)


# ΚΛΑΣΕΙΣ DRONES/ΥΔΑΤΙΝΩΝ ΟΧΗΜΑΤΩΝ

class Drone:
    """ Κλάση για drones και υδάτινα οχήματα. Κινείται με smooth animation και έχει περιορισμένη ορατότητα. """
    def __init__(self, x, y, color=COLOR_AGENT, speed=0.4):
        self.x, self.y = float(x), float(y)   # Θέση με δεκαδική ακρίβεια
        self.path = []                        # Διαδρομή προς τον στόχο
        self.vision_radius = 4                # Ακτίνα ορατότητας
        self.speed = speed                    # Ταχύτητα κίνησης
        self.color = color                    # Χρώμα οχήματος
    
    def set_path(self, path):
        """Ορίζει νέα διαδρομή για το drone"""
        self.path = list(path)
    
    def move(self):
        """ Κινεί το drone προς το επόμενο σημείο της διαδρομής. Χρησιμοποιεί smooth animation αντί για διακριτές κινήσεις. """
        if not self.path: 
            return
        
        # Στόχος: επόμενο σημείο της διαδρομής
        tx, ty = self.path[0]
        dx, dy = tx - self.x, ty - self.y
        dist = math.hypot(dx, dy)
        
        if dist < self.speed:
            # Άφιξη στον στόχο
            self.x, self.y = float(tx), float(ty)
            self.path.pop(0)
        else:
            # Κίνηση προς τον στόχο
            self.x += (dx / dist) * self.speed
            self.y += (dy / dist) * self.speed
    
    def draw(self, surface):
        """ Σχεδιάζει το drone ως κύκλο """
        center = (self.x * TILE_SIZE + TILE_SIZE / 2, 
                 self.y * TILE_SIZE + TILE_SIZE / 2)
        pygame.draw.circle(surface, self.color, center, TILE_SIZE // 3, 3)


# ΜΟΤΙΒΑ ΑΝΑΖΗΤΗΣΗΣ

class ExpandingSquarePattern:
    """ Μοτίβο αναζήτησης σε επεκτεινόμενα τετράγωνα. Ξεκινάει από το κέντρο και επεκτείνεται προς τα έξω."""
    def __init__(self, start_pos):
        self.start_pos = start_pos
    
    def get_path(self):
        """ Δημιουργεί διαδρομή αναζήτησης σε επεκτεινόμενα τετράγωνα. Επιστρέφει λίστα συντεταγμένων (x, y)."""
        path = [self.start_pos]
        last_pos = self.start_pos
        length = 1
        direction = 0  # 0: πάνω, 1: δεξιά, 2: κάτω, 3: αριστερά
        
        # Κατευθύνσεις κίνησης (dx, dy)
        moves = [(0, -1), (1, 0), (0, 1), (-1, 0)]
        
        while len(path) < 400:  # Όριο για αποφυγή άπειρων βρόχων
            # Κάθε τετράγωνο έχει δύο πλευρές ίσου μήκους
            for _ in range(2):
                for _ in range(length):
                    last_pos = (last_pos[0] + moves[direction][0], 
                               last_pos[1] + moves[direction][1])
                    
                    # Έλεγχος ορίων πλέγματος
                    if not (0 <= last_pos[0] < GRID_WIDTH_CELLS and 
                           0 <= last_pos[1] < GRID_HEIGHT_CELLS):
                        return path
                    
                    path.append(last_pos)
                
                # Αλλαγή κατεύθυνσης
                direction = (direction + 1) % 4
            
            # Αύξηση μήκους για το επόμενο τετράγωνο
            length += 1
        
        return path

class SectorSearchPattern:
    """ Μοτίβο αναζήτησης σε τομείς. Δημιουργεί ακτίνες από το κέντρο προς διάφορες κατευθύνσεις. """
    def __init__(self, start_pos, radius=15, num_sectors=12):
        self.start_pos = start_pos
        self.radius = radius
        self.num_sectors = num_sectors
    
    def get_path(self):
        """ Δημιουργεί διαδρομή αναζήτησης σε τομείς. Επιστρέφει λίστα συντεταγμένων (x, y). """
        path = [self.start_pos]
        sector_angle = 360 / self.num_sectors
        
        for i in range(self.num_sectors):
            # Υπολογισμός γωνίας για τον τρέχοντα τομέα
            angle = math.radians(i * sector_angle)
            
            # Υπολογισμός τελικού σημείου της ακτίνας
            end_x = round(max(0, min(GRID_WIDTH_CELLS - 1, 
                                   self.start_pos[0] + self.radius * math.cos(angle))))
            end_y = round(max(0, min(GRID_HEIGHT_CELLS - 1, 
                                   self.start_pos[1] - self.radius * math.sin(angle))))
            
            path.append((end_x, end_y))
            path.append(self.start_pos)  # Επιστροφή στο κέντρο
        
        return path

class ParallelSearchPattern:
    """ Μοτίβο αναζήτησης σε παράλληλες γραμμές. Δημιουργεί διαδρομή που καλύπτει όλο το πλέγμα σε παράλληλες γραμμές. """
    def __init__(self, start_pos):
        self.start_pos = start_pos
    
    def get_path(self):
        """ Δημιουργεί διαδρομή αναζήτησης σε παράλληλες γραμμές. Επιστρέφει λίστα συντεταγμένων (x, y) ταξινομημένες κατά απόσταση από την αρχή."""
        # Δημιουργία όλων των σημείων του πλέγματος
        path = []
        for y in range(GRID_HEIGHT_CELLS):
            if y % 2 == 0:
                # Ζυγές γραμμές: αριστερά προς δεξιά
                for x in range(GRID_WIDTH_CELLS):
                    path.append((x, y))
            else:
                # Περιττές γραμμές: δεξιά προς αριστερά
                for x in reversed(range(GRID_WIDTH_CELLS)):
                    path.append((x, y))
        
        # Ταξινόμηση κατά απόσταση από το σημείο εκκίνησης
        path.sort(key=lambda p: abs(p[0] - self.start_pos[0]) + abs(p[1] - self.start_pos[1]))
        
        return path


# ΣΥΝΑΡΤΗΣΕΙΣ ΔΙΑΧΕΙΡΙΣΗΣ ΣΤΑΤΙΣΤΙΚΩΝ

def load_ground_robot_stats():
    """ Φορτώνει τα στατιστικά του επίγειου ρομπότ από το JSON αρχείο. """
    stats_file = "ground_robot_stats.json"
    
    default_stats = {
        "total_missions": 0,
        "successful_missions": 0,
        "failed_missions": 0,
        "partial_missions": 0,
        "success_rate": 0.0,
        "average_time": 0.0,
        "best_time": float('inf'),
        "total_collectibles_found": 0,
        "total_enemies_encountered": 0
    }
    
    try:
        if os.path.exists(stats_file):
            with open(stats_file, 'r', encoding='utf-8') as f:
                stats = json.load(f)
                # Ενημέρωση με νέα πεδία αν δεν υπάρχουν
                for key, value in default_stats.items():
                    if key not in stats:
                        stats[key] = value
                return stats
        else:
            # Δημιουργία νέου αρχείου με προεπιλεγμένα στατιστικά
            save_ground_robot_stats(default_stats)
            return default_stats
    except (json.JSONDecodeError, IOError):
        return default_stats

def save_ground_robot_stats(stats):
    """ Αποθηκεύει τα στατιστικά του επίγειου ρομπότ στο JSON αρχείο. """
    stats_file = "ground_robot_stats.json"
    
    try:
        with open(stats_file, 'w', encoding='utf-8') as f:
            json.dump(stats, f, indent=2, ensure_ascii=False)
    except IOError as e:
        print(f"Σφάλμα αποθήκευσης στατιστικών: {e}")

def update_ground_robot_stats(result, mission_time, collectibles_found, enemies_count):
    """ Ενημερώνει τα στατιστικά του επίγειου ρομπότ. """
    stats = load_ground_robot_stats()
    
    # Ενημέρωση μετρητών αποστολών
    stats["total_missions"] += 1
    
    if result == "success":
        stats["successful_missions"] += 1
    elif result == "failed":
        stats["failed_missions"] += 1
    elif result == "partial":
        stats["partial_missions"] += 1
    
    # Υπολογισμός ποσοστού επιτυχίας
    if stats["total_missions"] > 0:
        stats["success_rate"] = (stats["successful_missions"] / stats["total_missions"]) * 100
    
    # Ενημέρωση χρόνου
    if stats["average_time"] == 0:
        stats["average_time"] = mission_time
    else:
        # Υπολογισμός νέου μέσου όρου
        total_time = stats["average_time"] * (stats["total_missions"] - 1) + mission_time
        stats["average_time"] = total_time / stats["total_missions"]
    
    # Καλύτερος χρόνος
    if result == "success" and mission_time < stats["best_time"]:
        stats["best_time"] = mission_time
    
    # Ενημέρωση συλλεγμένων αντικειμένων και εχθρών
    stats["total_collectibles_found"] += collectibles_found
    stats["total_enemies_encountered"] += enemies_count
    
    # Αποθήκευση
    save_ground_robot_stats(stats)
    
    return stats


# ΚΥΡΙΑ ΚΛΑΣΗ ΠΑΙΧΝΙΔΙΟΥ

class Game:
    """ Κύρια κλάση διαχείρισης του παιχνιδιού. Χρησιμοποιεί Singleton pattern για εύκολη πρόσβαση από άλλες κλάσεις. """
    _instance = None
    
    @staticmethod
    def get_instance():
        """Επιστρέφει την μοναδική instance της κλάσης Game"""
        return Game._instance
    
    def __init__(self, **kwargs):
        """ Αρχικοποίηση του παιχνιδιού. kwargs: παράμετροι αποστολής (game_mode, num_patrol, κλπ) """
        Game._instance = self
        self.mission_settings = kwargs
        self.game_mode = self.mission_settings.get('game_mode', 'ground')
        
        # Αρχικοποίηση Pygame
        pygame.init()
        self.screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
        pygame.display.set_caption(f"SAR - {self.game_mode.replace('_', ' ').title()}")
        self.clock = pygame.time.Clock()
        self.font = pygame.font.SysFont("Arial", 20)
        self.running = True
        self.paused = False  # Κατάσταση παύσης
        
        # Εκκίνηση νέας αποστολής
        self.setup_new_mission()
    
    def is_dead_end(self, x, y):
        """ Ελέγχει αν ένα κελί είναι αδιέξοδο. Αδιέξοδο = έχει μόνο μία έξοδο (εκτός αν είναι στόχος). """
        # Λίστα όλων των στόχων
        targets = getattr(self, 'collectibles', []) + getattr(self, 'land_collectibles', [])
        if (x, y) in targets: 
            return False
        
        # Μέτρηση ελεύθερων γειτονικών κελιών
        free_neighbors = 0
        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            nx, ny = x + dx, y + dy
            if (0 <= nx < GRID_WIDTH_CELLS and 
                0 <= ny < GRID_HEIGHT_CELLS and 
                self.game_map[ny][nx] != TILE_TYPE_OBSTACLE):
                free_neighbors += 1
        
        return free_neighbors <= 1  # Αδιέξοδο αν έχει 1 ελεύθερο γείτονα
    
    def setup_new_mission(self):
        """ Αρχικοποίηση νέας αποστολής. Επαναφέρει όλες τις μεταβλητές και καλεί την κατάλληλη setup function. """
        # Επαναφορά κατάστασης αποστολής
        self.mission_complete = False
        self.mission_failed = False
        self.mission_partial = False
        self.paused = False  # Επαναφορά κατάστασης παύσης
        self.start_time = pygame.time.get_ticks()
        self.final_time = 0
        
        # Λεξικό με όλες τις setup functions
        setups = {
            "ground": self.setup_ground,
            "single_drone": self.setup_single_drone,
            "single_aquatic": self.setup_single_aquatic,
            "coop_ground_drone": self.setup_coop_ground_drone,
            "drone_aquatic": self.setup_drone_aquatic,
            "all_agents": self.setup_all_agents
        }
        
        # Κλήση της κατάλληλης setup function
        setups.get(self.game_mode, self.setup_ground)()
    
    def setup_ground(self):
        """ Αρχικοποίηση αποστολής επίγειου ρομπότ. Δημιουργεί χάρτη, εχθρούς, αντικείμενα συλλογής. """
        # Αρχικοποίηση μεταβλητών αποστολής
        self.last_positions = collections.deque(maxlen=8)  # Ιστορικό θέσεων
        self.loop_counter = 0                              # Μετρητής βρόχων
        self.target_cooldown = {}                          # Cooldown για στόχους
        self.target_attempts = {}                          # Προσπάθειες για κάθε στόχο
        self.current_target = None                         # Τρέχων στόχος
        self.collectibles = []                             # Λίστα αντικειμένων συλλογής
        self.dynamic_obstacles = []                        # Λίστα εχθρών
        
        # Δημιουργία τυχαίου χάρτη
        # 0=ελεύθερο, 1=εμπόδιο, 2=δύσκολο, 3=κίνδυνος
        self.game_map = [[random.choice([0, 0, 0, 0, 1, 1, 2, 3]) 
                         for _ in range(GRID_WIDTH_CELLS)] 
                        for _ in range(GRID_HEIGHT_CELLS)]
        
        # Δημιουργία επίγειου ρομπότ
        self.agent = Agent(1, 1)
        
        # Ρύθμιση αριθμού εχθρών
        self.num_patrol = self.mission_settings.get("num_patrol", 
                                                   random.randint(NUM_PATROL_MIN, NUM_PATROL_MAX))
        self.num_aggressive = self.mission_settings.get("num_aggressive", 
                                                       random.randint(NUM_AGGRESSIVE_MIN, NUM_AGGRESSIVE_MAX))
        
        # Δημιουργία εχθρών
        for _ in range(self.num_patrol): 
            self.add_obstacle(PatrolObstacle)
        for _ in range(self.num_aggressive): 
            self.add_obstacle(AggressiveObstacle)
        
        # Δημιουργία αντικειμένων συλλογής
        while len(self.collectibles) < 10:
            x, y = random.randint(1, GRID_WIDTH_CELLS - 2), random.randint(1, GRID_HEIGHT_CELLS - 2)
            if (self.game_map[y][x] == 0 and 
                (x, y) != (1, 1) and 
                not any((x, y) == (o.x, o.y) for o in self.dynamic_obstacles)):
                self.collectibles.append((x, y))
        
        self.total_collectibles = len(self.collectibles)
    
    def setup_single_vehicle(self, is_aquatic):
        """ Αρχικοποίηση αποστολής ενός οχήματος (drone ή υδάτινο). is_aquatic: True για υδάτινο όχημα, False για drone """
        # Δημιουργία χάρτη (νερό ή ξηρά)
        if is_aquatic:
            self.game_map = [[TILE_TYPE_WATER for _ in range(GRID_WIDTH_CELLS)] 
                           for _ in range(GRID_HEIGHT_CELLS)]
        else:
            self.game_map = [[TILE_TYPE_FREE for _ in range(GRID_WIDTH_CELLS)] 
                           for _ in range(GRID_HEIGHT_CELLS)]
        
        # Προσθήκη τυχαίων εμποδίων
        for _ in range(70):
            x, y = random.randint(0, GRID_WIDTH_CELLS - 1), random.randint(0, GRID_HEIGHT_CELLS - 1)
            self.game_map[y][x] = TILE_TYPE_OBSTACLE
        
        # Δημιουργία οχήματος
        start_pos = (GRID_WIDTH_CELLS // 2, GRID_HEIGHT_CELLS // 2)
        color = COLOR_AQUATIC_AGENT if is_aquatic else COLOR_AGENT
        self.vehicle = Drone(start_pos[0], start_pos[1], color=color)
        
        # Εύρεση τυχαίου στόχου
        while True:
            self.target = (random.randint(1, GRID_WIDTH_CELLS - 2), 
                          random.randint(1, GRID_HEIGHT_CELLS - 2))
            if (self.game_map[self.target[1]][self.target[0]] != TILE_TYPE_OBSTACLE and 
                math.hypot(start_pos[0] - self.target[0], start_pos[1] - self.target[1]) > 5):
                break
        
        # Αρχικοποίηση fog of war
        self.revealed_map = [[False for _ in range(GRID_WIDTH_CELLS)] 
                           for _ in range(GRID_HEIGHT_CELLS)]
        self.phase = 'search'
        self.target_found_message = ""
        
        # Επιλογή μοτίβου αναζήτησης
        pattern_str = self.mission_settings.get('pattern', 'parallel_search')
        patterns = {
            "expanding_square": ExpandingSquarePattern,
            "sector_search": SectorSearchPattern,
            "parallel_search": ParallelSearchPattern
        }
        self.vehicle.set_path(patterns[pattern_str]((self.vehicle.x, self.vehicle.y)).get_path())
    
    def setup_single_drone(self): 
        """Αρχικοποίηση αποστολής drone"""
        self.setup_single_vehicle(is_aquatic=False)
    
    def setup_single_aquatic(self): 
        """Αρχικοποίηση αποστολής υδάτινου οχήματος"""
        self.setup_single_vehicle(is_aquatic=True)
    
    def setup_coop_ground_drone(self):
        """ Αρχικοποίηση συνεργατικής αποστολής drone + επίγειο ρομπότ. Το drone σαρώνει πρώτα το χάρτη, μετά το ρομπότ συλλέγει αντικείμενα. """
        self.phase = "drone_scan"
        self.agent = Agent(1, 1)
        self.current_target = None
        self.last_positions = collections.deque(maxlen=8)
        self.loop_counter = 0
        self.target_cooldown = {}
        self.target_attempts = {}
        
        # Δημιουργία απλού χάρτη (χωρίς εχθρούς)
        self.game_map = [[random.choice([0, 0, 0, 1, 2]) for _ in range(GRID_WIDTH_CELLS)] 
                        for _ in range(GRID_HEIGHT_CELLS)]
        self.collectibles = []
        
        # Δημιουργία αντικειμένων συλλογής
        while len(self.collectibles) < 7:
            x, y = random.randint(1, GRID_WIDTH_CELLS - 2), random.randint(1, GRID_HEIGHT_CELLS - 2)
            if self.game_map[y][x] == 0:
                self.collectibles.append((x, y))
        
        self.total_collectibles = len(self.collectibles)
        
        # Δημιουργία drone
        start_pos = (GRID_WIDTH_CELLS // 2, GRID_HEIGHT_CELLS // 2)
        self.drone = Drone(start_pos[0], start_pos[1])
        
        # Επιλογή μοτίβου αναζήτησης για το drone
        pattern_str = self.mission_settings.get('pattern', 'parallel_search')
        patterns = {
            "expanding_square": ExpandingSquarePattern,
            "sector_search": SectorSearchPattern,
            "parallel_search": ParallelSearchPattern
        }
        self.drone.set_path(patterns[pattern_str]((self.drone.x, self.drone.y)).get_path())
        
        # Αρχικοποίηση fog of war
        self.revealed_map = [[False] * GRID_WIDTH_CELLS for _ in range(GRID_HEIGHT_CELLS)]
    
    def setup_drone_aquatic(self):
        """ Αρχικοποίηση αποστολής drone + υδάτινο όχημα. Το drone σαρώνει το χάρτη, το υδάτινο όχημα πηγαίνει στον στόχο. """
        self.phase = "drone_scan"
        
        # Δημιουργία υδάτινου χάρτη
        self.game_map = [[TILE_TYPE_WATER] * GRID_WIDTH_CELLS for _ in range(GRID_HEIGHT_CELLS)]
        for _ in range(70):
            x, y = random.randint(0, GRID_WIDTH_CELLS - 1), random.randint(0, GRID_HEIGHT_CELLS - 1)
            self.game_map[y][x] = TILE_TYPE_OBSTACLE
        
        # Δημιουργία οχημάτων
        self.drone = Drone(GRID_WIDTH_CELLS // 2, 2)
        self.aquatic = Drone(GRID_WIDTH_CELLS // 2, GRID_HEIGHT_CELLS - 2, color=COLOR_AQUATIC_AGENT)
        
        # Εύρεση στόχου
        while True:
            self.target = (random.randint(1, GRID_WIDTH_CELLS - 2), 
                          random.randint(1, GRID_HEIGHT_CELLS - 2))
            if self.game_map[self.target[1]][self.target[0]] != 1:
                break
        
        # Επιλογή μοτίβων αναζήτησης
        patterns = {
            "expanding_square": ExpandingSquarePattern,
            "sector_search": SectorSearchPattern,
            "parallel_search": ParallelSearchPattern
        }
        self.drone.set_path(patterns[self.mission_settings.get('drone_pattern')]((self.drone.x, self.drone.y)).get_path())
        self.aquatic.set_path(patterns[self.mission_settings.get('aquatic_pattern')]((self.aquatic.x, self.aquatic.y)).get_path())
        
        # Αρχικοποίηση fog of war
        self.revealed_map = [[False] * GRID_WIDTH_CELLS for _ in range(GRID_HEIGHT_CELLS)]
    
    def setup_all_agents(self):
        """ Αρχικοποίηση πλήρους αποστολής με όλα τα οχήματα. Χάρτης με λίμνη: άνω μέρος ξηρά, κάτω μέρος νερό. """
        self.phase = "drone_scan"
        self.agent = Agent(1, 1)
        self.current_target = None
        self.last_positions = collections.deque(maxlen=8)
        self.loop_counter = 0
        self.target_cooldown = {}
        self.target_attempts = {}
        
        # Δημιουργία χάρτη με λίμνη
        lake_level = GRID_HEIGHT_CELLS // 2
        self.game_map = [[TILE_TYPE_WATER if y > lake_level else TILE_TYPE_FREE 
                         for x in range(GRID_WIDTH_CELLS)] 
                        for y in range(GRID_HEIGHT_CELLS)]
        
        # Προσθήκη εμποδίων
        for _ in range(80):
            x, y = random.randint(0, GRID_WIDTH_CELLS - 1), random.randint(0, GRID_HEIGHT_CELLS - 1)
            self.game_map[y][x] = TILE_TYPE_OBSTACLE
        
        # Δημιουργία οχημάτων
        self.drone = Drone(GRID_WIDTH_CELLS // 2, 2)
        self.aquatic = Drone(GRID_WIDTH_CELLS // 2, GRID_HEIGHT_CELLS - 2, color=COLOR_AQUATIC_AGENT)
        
        # Δημιουργία αντικειμένων συλλογής
        self.land_collectibles = []
        self.water_collectibles = []
        
        # Αντικείμενα στη ξηρά
        while len(self.land_collectibles) < 3:
            x, y = random.randint(1, GRID_WIDTH_CELLS - 2), random.randint(1, lake_level)
            if self.game_map[y][x] == 0:
                self.land_collectibles.append((x, y))
        
        # Αντικείμενα στο νερό
        while len(self.water_collectibles) < 2:
            x, y = random.randint(1, GRID_WIDTH_CELLS - 2), random.randint(lake_level + 1, GRID_HEIGHT_CELLS - 2)
            if self.game_map[y][x] == 4:
                self.water_collectibles.append((x, y))
        
        self.total_collectibles = len(self.land_collectibles) + len(self.water_collectibles)
        
        # Επιλογή μοτίβων αναζήτησης
        pattern_str = self.mission_settings.get('pattern', 'parallel_search')
        patterns = {
            "expanding_square": ExpandingSquarePattern,
            "sector_search": SectorSearchPattern,
            "parallel_search": ParallelSearchPattern
        }
        self.drone.set_path(patterns[pattern_str]((self.drone.x, self.drone.y)).get_path())
        self.aquatic.set_path(ParallelSearchPattern((self.aquatic.x, self.aquatic.y)).get_path())
        
        # Αρχικοποίηση fog of war
        self.revealed_map = [[False] * GRID_WIDTH_CELLS for _ in range(GRID_HEIGHT_CELLS)]
    
    def add_obstacle(self, obs_type):
        """ Προσθήκη εχθρού στο χάρτη. obs_type: τύπος εχθρού (PatrolObstacle ή AggressiveObstacle) """
        for _ in range(100):  # Μέγιστος αριθμός προσπαθειών
            x, y = random.randint(1, GRID_WIDTH_CELLS - 2), random.randint(1, GRID_HEIGHT_CELLS - 2)
            
            # Έλεγχος εγκυρότητας θέσης
            if (self.game_map[y][x] == 0 and 
                math.hypot(x - 1, y - 1) > 6 and  # Απόσταση από τον παίκτη
                not any(math.hypot(x - o.x, y - o.y) <= 2 for o in self.dynamic_obstacles)):  # Απόσταση από άλλους εχθρούς
                
                # Δημιουργία εχθρού
                if obs_type == AggressiveObstacle:
                    obstacle = obs_type(x, y, self.game_map, self.agent)
                else:
                    obstacle = obs_type(x, y, self.game_map)
                
                self.dynamic_obstacles.append(obstacle)
                return
    
    def run(self):
        """ Κύριος βρόχος του παιχνιδιού. Ενημερώνει την κατάσταση, σχεδιάζει την οθόνη, χειρίζεται events. """
        self.back_to_menu = False
        
        # Λεξικό με update functions για κάθε τρόπο παιχνιδιού
        updates = {
            "ground": self.update_ground,
            "single_drone": self.update_single_vehicle,
            "single_aquatic": self.update_single_vehicle,
            "coop_ground_drone": self.update_coop_ground_drone,
            "drone_aquatic": self.update_drone_aquatic,
            "all_agents": self.update_all_agents
        }
        
        # Εύρεση αρχικής διαδρομής
        self.find_path()
        
        # Κύριος βρόχος παιχνιδιού
        while self.running:
            self.handle_events()
            
            # Ενημέρωση κατάστασης μόνο αν η αποστολή δεν έχει τελειώσει και δεν είναι σε παύση
            if not any([self.mission_complete, self.mission_failed, self.mission_partial]) and not self.paused:
                updates.get(self.game_mode, self.update_ground)()
            
            # Σχεδιασμός και ενημέρωση οθόνης
            self.draw()
            self.clock.tick(7)  # 7 FPS
            
            # Χείριση τέλους αποστολής
            if any([self.mission_complete, self.mission_failed, self.mission_partial]):
                self.handle_end_of_mission_events()
        
        return self.back_to_menu
    
    def handle_events(self):
        """
        Χείριση events από το χρήστη.
        ESC: επιστροφή στο μενού
        R: επανεκκίνηση αποστολής
        SPACE: παύση/συνέχεια παιχνιδιού
        """
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False
                self.back_to_menu = False
            
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    self.running = False
                    self.back_to_menu = True
                
                if event.key == pygame.K_SPACE:
                    # Παύση/συνέχεια παιχνιδιού
                    self.paused = not self.paused
                
                if (event.key == pygame.K_r and 
                    not any([self.mission_complete, self.mission_failed, self.mission_partial])):
                    self.setup_new_mission()
                    self.find_path()
    
    def handle_end_of_mission_events(self):
        """
        Χείριση events μετά το τέλος της αποστολής.
        R: νέα αποστολή
        ESC: επιστροφή στο μενού
        """
        waiting = True
        while waiting and self.running:
            self.draw()
            
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.running = False
                    waiting = False
                
                if event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_r:
                        self.setup_new_mission()
                        self.find_path()
                        waiting = False
                    
                    if event.key == pygame.K_ESCAPE:
                        self.running = False
                        self.back_to_menu = True
                        waiting = False
            
            self.clock.tick(10)
    
    def update_ground(self):
        """ Ενημέρωση κατάστασης αποστολής επίγειου ρομπότ. Χειρίζεται κίνηση εχθρών, αποφυγή απειλών, συλλογή αντικειμένων. """
        pos = (self.agent.x, self.agent.y)
        
        # Έλεγχος για στόχους που είναι πολύ δύσκολοι
        if self.current_target and self.target_attempts.get(self.current_target, 0) > 10:
            self.target_cooldown[self.current_target] = 25
            self.current_target = None
            self.agent.path = []
        
        # Ενημέρωση εχθρών
        dynamic_obstacles = getattr(self, 'dynamic_obstacles', [])
        for obs in dynamic_obstacles:
            obs.update([TILE_TYPE_FREE, TILE_TYPE_DIFFICULT, TILE_TYPE_HAZARD])
            
            # Έλεγχος σύγκρουσης με εχθρό
            if (obs.x, obs.y) == pos:
                self.end_mission("failed")
                return
        
        # Εύρεση απειλών (εχθρών που μπορούν να φτάσουν τον παίκτη)
        threats = []
        for o in dynamic_obstacles:
            turns_to_reach = self.turns_to_reach((o.x, o.y), pos)
            # Βελτιωμένη λογική απειλών
            if isinstance(o, AggressiveObstacle):
                # Για επιθετικούς εχθρούς: μεγαλύτερο εύρος απειλής
                max_turns = o.detection_radius + 2
                # Επιπλέον έλεγχος αν ο εχθρός είναι σε λειτουργία attack
                if o.mode == 'attack':
                    max_turns += 1
            else:
                # Για εχθρούς περιπολίας: μικρότερο εύρος απειλής
                max_turns = 3
            
            if turns_to_reach <= max_turns:
                threats.append(o)
        
        if threats:
            # Προσπάθεια "brave path" - συλλογή αντικειμένου παρά την απειλή
            brave_path = self.find_brave_path(threats)
            if brave_path:
                self.current_target = brave_path[-1]
                self.agent.set_path(brave_path)
            else:
                # Αναζήτηση διαδρομής διαφυγής
                escape_path = self.find_escape_path(threats, dynamic_obstacles)
                if escape_path:
                    self.current_target = None
                    self.agent.set_path(escape_path)
                else:
                    # Εύρεση καλύτερης κίνησης για αποφυγή
                    best_move, max_dist = None, -1
                    closest_threat = min(threats, key=lambda t: math.hypot(pos[0] - t.x, pos[1] - t.y))
                    
                    for dx, dy in [(x, y) for x in [-1, 0, 1] for y in [-1, 0, 1] if not (x == 0 and y == 0)]:
                        nx, ny = pos[0] + dx, pos[1] + dy
                        if (0 <= nx < GRID_WIDTH_CELLS and 
                            0 <= ny < GRID_HEIGHT_CELLS and 
                            self.game_map[ny][nx] not in [1, 4] and 
                            not any((nx, ny) == (o.x, o.y) for o in dynamic_obstacles)):
                            
                            dist = math.hypot(nx - closest_threat.x, ny - closest_threat.y)
                            if dist > max_dist:
                                max_dist = dist
                                best_move = (nx, ny)
                    
                    if best_move:
                        self.agent.set_path([best_move])
                    else:
                        self.agent.path = []
                        self.agent.wait_timer = 1
        
        elif not self.agent.path or self.current_target not in self.collectibles:
            # Αναζήτηση νέου στόχου αν δεν υπάρχει απειλή
            self.find_path_to_collectible(self.collectibles, [TILE_TYPE_WATER, TILE_TYPE_OBSTACLE])
        
        # Κίνηση του ρομπότ
        self.agent.move()
        
        # Έλεγχος συλλογής αντικειμένου
        if (self.agent.x, self.agent.y) in self.collectibles:
            self.collectibles.remove((self.agent.x, self.agent.y))
            self.agent.score += 10
            
            # Καθαρισμός από target_attempts
            if (self.agent.x, self.agent.y) in self.target_attempts:
                del self.target_attempts[(self.agent.x, self.agent.y)]
            
            self.current_target = None
            self.agent.path = []
            
            # Έλεγχος ολοκλήρωσης αποστολής
            if not self.collectibles:
                self.end_mission("success")
    
    def update_single_vehicle(self):
        """ Ενημέρωση κατάστασης αποστολής ενός οχήματος (drone ή υδάτινο). Χειρίζεται τις φάσεις αναζήτησης και μετάβασης στον στόχο. """
        if not self.vehicle.path:
            self.end_mission("failed")
            return
        
        if self.phase == 'search':
            # Φάση αναζήτησης: κίνηση σύμφωνα με το μοτίβο αναζήτησης
            self.vehicle.move()
            vx, vy = int(self.vehicle.x), int(self.vehicle.y)
            radius = int(self.vehicle.vision_radius)
            
            # Ενημέρωση fog of war - αποκάλυψη περιοχής γύρω από το drone
            for y in range(max(0, vy - radius), min(GRID_HEIGHT_CELLS, vy + radius + 1)):
                for x in range(max(0, vx - radius), min(GRID_WIDTH_CELLS, vx + radius + 1)):
                    if math.hypot(x - self.vehicle.x, y - self.vehicle.y) <= self.vehicle.vision_radius:
                        self.revealed_map[y][x] = True
            
            # Έλεγχος αν ο στόχος έχει εντοπιστεί
            if self.revealed_map[self.target[1]][self.target[0]]:
                self.phase = 'goto_target'
                self.target_found_message = f"Target spotted at: {self.target}"
                
                # Εύρεση διαδρομής προς τον στόχο
                start_node = (int(self.vehicle.x), int(self.vehicle.y))
                path, _ = self.a_star(start_node, self.target, [TILE_TYPE_OBSTACLE])
                if path:
                    self.vehicle.set_path(path)
                else:
                    self.end_mission("failed")
        
        elif self.phase == 'goto_target':
            # Φάση μετάβασης στον στόχο
            self.vehicle.move()
            if math.hypot(self.vehicle.x - self.target[0], self.vehicle.y - self.target[1]) < 1.0:
                self.end_mission("success")
    
    def drone_scan_phase(self, drone, revealed_map):
        """ Ενημέρωση φάσης σάρωσης drone. Επιστρέφει True αν η σάρωση ολοκληρώθηκε. """
        if drone.path:
            drone.move()
            
            # Ενημέρωση fog of war για όλο το χάρτη
            for y in range(GRID_HEIGHT_CELLS):
                for x in range(GRID_WIDTH_CELLS):
                    if not revealed_map[y][x] and math.hypot(x - drone.x, y - drone.y) <= drone.vision_radius:
                        revealed_map[y][x] = True
            return False
        return True
    
    def update_coop_ground_drone(self):
        """ Ενημέρωση συνεργατικής αποστολής drone + επίγειο ρομπότ. Πρώτα το drone σαρώνει, μετά το ρομπότ συλλέγει αντικείμενα. """
        if self.phase == "drone_scan":
            if self.drone_scan_phase(self.drone, self.revealed_map):
                self.phase = "ground_search"
        elif self.phase == "ground_search":
            if not self.collectibles:
                self.end_mission("success")
                return
            self.update_ground()
    
    def update_drone_aquatic(self):
        """ Ενημέρωση αποστολής drone + υδάτινο όχημα. Πρώτα το drone σαρώνει, μετά το υδάτινο όχημα πηγαίνει στον στόχο. """
        if self.phase == "drone_scan":
            if self.drone_scan_phase(self.drone, self.revealed_map):
                self.phase = "aquatic_search"
        elif self.phase == "aquatic_search":
            if self.aquatic.path:
                self.aquatic.move()
                if math.hypot(self.aquatic.x - self.target[0], self.aquatic.y - self.target[1]) <= 1.0:
                    self.end_mission("success")
            else:
                self.end_mission("failed")
    
    def update_all_agents(self):
        """ Ενημέρωση πλήρους αποστολής με όλα τα οχήματα. Συντονισμένη αναζήτηση σε ξηρά και νερό. """
        if self.phase == "drone_scan":
            if self.drone_scan_phase(self.drone, self.revealed_map):
                self.phase = "coordinated_search"
        elif self.phase == "coordinated_search":
            # Ενημέρωση επίγειου ρομπότ για αντικείμενα στη ξηρά
            if self.land_collectibles:
                self.collectibles = self.land_collectibles
                self.dynamic_obstacles = []
                self.update_ground()
                self.land_collectibles = self.collectibles
            
            # Ενημέρωση υδάτινου οχήματος για αντικείμενα στο νερό
            if self.water_collectibles:
                self.aquatic.move()
                collected = next((t for t in self.water_collectibles 
                                if math.hypot(self.aquatic.x - t[0], self.aquatic.y - t[1]) < 1.5), None)
                if collected:
                    self.water_collectibles.remove(collected)
            
            # Έλεγχος ολοκλήρωσης αποστολής
            if not self.land_collectibles and not self.water_collectibles:
                self.end_mission("success")
    
    def find_path(self):
        """ Εύρεση αρχικής διαδρομής για το επίγειο ρομπότ. """
        if hasattr(self, 'agent') and self.game_mode == "ground":
            self.find_path_to_collectible(self.collectibles, [TILE_TYPE_WATER, TILE_TYPE_OBSTACLE])
    
    def find_path_to_collectible(self, targets, restricted_tiles):
        """ Εύρεση διαδρομής προς το καλύτερο αντικείμενο συλλογής. targets: λίστα αντικειμένων συλλογής restricted_tiles: τύποι κελιών που δεν μπορεί να περάσει """
        if not targets:
            return
        
        pos = (self.agent.x, self.agent.y)
        cost_map = self.create_dynamic_cost_map()
        available = [t for t in targets if t not in self.target_cooldown]
        best_path, best_cost, best_target = None, float('inf'), None
        
        # Αναζήτηση καλύτερου στόχου
        for target in available:
            path, cost = self.a_star(pos, target, restricted_tiles, cost_map)
            if path and cost < best_cost:
                best_path, best_cost, best_target = path, cost, target
        
        if best_path:
            self.current_target = best_target
            self.agent.set_path(best_path)
            self.target_attempts[best_target] = self.target_attempts.get(best_target, 0) + 1
        else:
            self.agent.path = []
            self.current_target = None
    
    def find_escape_path(self, threats, dynamic_obstacles):
        """ Εύρεση διαδρομής διαφυγής από απειλές. 
        threats: λίστα απειλών
        dynamic_obstacles: λίστα εχθρών
        """
        pos = (self.agent.x, self.agent.y)
        cost_map = self.create_dynamic_cost_map(escape=True)
        best_path, best_score = None, -float('inf')
        
        # Εύρεση υποψηφίων σημείων διαφυγής
        candidates = []
        for r in range(2, 8):
            for i in range(-r, r + 1):
                for j in range(-r, r + 1):
                    if (abs(i) + abs(j) == r and 
                        0 <= pos[0] + i < GRID_WIDTH_CELLS and 
                        0 <= pos[1] + j < GRID_HEIGHT_CELLS and 
                        self.game_map[pos[1] + j][pos[0] + i] not in [1, 4]):
                        candidates.append((pos[0] + i, pos[1] + j))
        
        # Αξιολόγηση κάθε υποψηφίου σημείου
        for spot in candidates:
            if any(spot == (o.x, o.y) for o in dynamic_obstacles):
                continue
            
            path, cost = self.a_star(pos, spot, [TILE_TYPE_WATER, TILE_TYPE_OBSTACLE], cost_map)
            if path and not any(s == (o.x, o.y) for s in path for o in dynamic_obstacles):
                score = min(self.turns_to_reach((t.x, t.y), spot) for t in threats) * 1.5 - len(path)
                if score > best_score:
                    best_path, best_score = path, score
        
        return best_path
    
    def find_brave_path(self, threats):
        """
        Εύρεση "θαρραλέου" μονοπατιού για συλλογή αντικειμένου παρά την απειλή.
        threats: λίστα απειλών
        """
        agent_pos = (self.agent.x, self.agent.y)
        aggressive_threats = [t for t in threats if isinstance(t, AggressiveObstacle)]
        if not aggressive_threats:
            return None
        
        # Ταξινόμηση αντικειμένων κατά απόσταση
        sorted_collectibles = sorted(self.collectibles, 
                                   key=lambda c: abs(c[0] - agent_pos[0]) + abs(c[1] - agent_pos[1]))
        
        for target in sorted_collectibles:
            if self.is_dead_end(target[0], target[1]):
                continue
            
            agent_steps = self.turns_to_reach(agent_pos, target)
            if agent_steps == float('inf'):
                continue
            
            enemy_steps = min(self.turns_to_reach((t.x, t.y), target) for t in aggressive_threats)
            
            # Έλεγχος αν ο παίκτης μπορεί να φτάσει πριν τον εχθρό
            if agent_steps < enemy_steps - 1:
                cost_map = self.create_dynamic_cost_map(escape=True)
                path, cost = self.a_star(agent_pos, target, [TILE_TYPE_WATER, TILE_TYPE_OBSTACLE], cost_map)
                if path:
                    return path
        
        return None
    
    def create_dynamic_cost_map(self, escape=False):
        """ 
        Δημιουργία δυναμικού χάρτη κόστους που λαμβάνει υπόψη εχθρούς και αδιέξοδα.
        escape: True αν είναι για διαφυγή, False για κανονική κίνηση
        """
        # Βασικό κόστος βάσει τύπου εδάφους
        cost_map = [[1 + self.game_map[y][x] * 5 for x in range(GRID_WIDTH_CELLS)] 
                   for y in range(GRID_HEIGHT_CELLS)]
        
        # Ποινή για αδιέξοδα
        for y in range(GRID_HEIGHT_CELLS):
            for x in range(GRID_WIDTH_CELLS):
                if self.is_dead_end(x, y):
                    penalty = 4000 if escape else 200
                    cost_map[y][x] += penalty
        
        # Ποινές για εχθρούς
        for obs in getattr(self, 'dynamic_obstacles', []):
            if isinstance(obs, AggressiveObstacle):
                # Επιθετικοί εχθροί: βελτιωμένη αποφυγή με μεγαλύτερο εύρος επιρροής
                r = obs.detection_radius
                # Αύξηση του εύρους επιρροής για καλύτερη αποφυγή
                extended_radius = r + 2 if not escape else r + 4
                
                for dy in range(-extended_radius, extended_radius + 1):
                    for dx in range(-extended_radius, extended_radius + 1):
                        nx, ny = obs.x + dx, obs.y + dy
                        if 0 <= nx < GRID_WIDTH_CELLS and 0 <= ny < GRID_HEIGHT_CELLS:
                            dist = abs(dx) + abs(dy)
                            
                            # Κλιμακωτό κόστος βάσει απόστασης
                            if dist == 0:
                                cost_map[ny][nx] += 8000  # Αυξημένο κόστος στο κέντρο
                            elif dist <= 1:
                                cost_map[ny][nx] += 3000  # Υψηλό κόστος γύρω από εχθρό
                            elif dist <= 2:
                                cost_map[ny][nx] += 1500  # Μεσαίο κόστος
                            elif dist <= r:
                                cost_map[ny][nx] += 800   # Χαμηλότερο κόστος στην περιοχή εντοπισμού
                            elif dist <= extended_radius:
                                cost_map[ny][nx] += 300   # Πολύ χαμηλό κόστος στο εκτεταμένο εύρος
                            
                            # Επιπλέον ποινή αν ο εχθρός είναι σε λειτουργία attack
                            if obs.mode == 'attack':
                                cost_map[ny][nx] += 2000
            else:
                # Εχθροί περιπολίας: κόστος γύρω από τη θέση τους
                cost_map[obs.y][obs.x] += 2500
                for dx, dy in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
                    nx, ny = obs.x + dx, obs.y + dy
                    if (0 <= nx < GRID_WIDTH_CELLS and 
                        0 <= ny < GRID_HEIGHT_CELLS and 
                        self.game_map[ny][nx] not in [1, 4]):
                        cost_map[ny][nx] += 1200
        
        return cost_map
    
    def turns_to_reach(self, start, end):
        """ Υπολογισμός αριθμού βημάτων για να φτάσει από start σε end. """
        path, cost = self.a_star(start, end, [1, 4])
        return cost
    
    def a_star(self, start, end, restricted, cost_map=None):
        """
        Βελτιωμένη υλοποίηση A* αλγορίθμου για εύρεση βέλτιστης διαδρομής.
        start: σημείο εκκίνησης
        end: σημείο τερματισμού
        restricted: τύποι κελιών που δεν μπορεί να περάσει
        cost_map: χάρτης κόστους (προαιρετικός)
        """
        if cost_map is None:
            cost_map = [[1] * GRID_WIDTH_CELLS for _ in range(GRID_HEIGHT_CELLS)]
        
        nodes = [(0, Node(start))]
        closed = set()
        g_costs = {start: 0}
        
        while nodes:
            _, c_node = heapq.heappop(nodes)
            
            if c_node.position == end:
                # Κατασκευή διαδρομής
                path = []
                p = c_node
                while p:
                    path.append(p.position)
                    p = p.parent
                return path[::-1], g_costs.get(end, float('inf'))
            
            if c_node.position in closed:
                continue
            closed.add(c_node.position)
            
            # Εξερεύνηση γειτονικών κελιών (μόνο καρδινάλες κινήσεις)
            # Καρδινάλες κινήσεις (πάνω, κάτω, αριστερά, δεξιά)
            cardinal_moves = [(0, -1), (0, 1), (-1, 0), (1, 0)]
            
            for move in cardinal_moves:
                pos = (c_node.position[0] + move[0], c_node.position[1] + move[1])
                
                if (not (0 <= pos[0] < GRID_WIDTH_CELLS and 0 <= pos[1] < GRID_HEIGHT_CELLS) or
                    self.game_map[pos[1]][pos[0]] in restricted or
                    pos in closed):
                    continue
                
                # Υπολογισμός κόστους κίνησης (πάντα 1.0 για καρδινάλες κινήσεις)
                new_g = g_costs.get(c_node.position, float('inf')) + cost_map[pos[1]][pos[0]]
                
                if new_g < g_costs.get(pos, float('inf')):
                    g_costs[pos] = new_g
                    # Manhattan distance ευρετική (πιο κατάλληλη για καρδινάλες κινήσεις)
                    h = abs(pos[0] - end[0]) + abs(pos[1] - end[1])
                    heapq.heappush(nodes, (new_g + h, Node(pos, c_node)))
        
        return None, float('inf')
    
    def end_mission(self, result):
        """
        Τερματισμός αποστολής με αποτέλεσμα.
        result: "success", "failed", ή "partial"
        """
        if self.mission_complete or self.mission_failed or self.mission_partial:
            return
        
        self.final_time = (pygame.time.get_ticks() - self.start_time) // 1000
        
        # Έλεγχος για μερική επιτυχία
        if result == "failed" and hasattr(self, 'total_collectibles') and self.total_collectibles > 0:
            collectibles_left = len(getattr(self, 'collectibles', []))
            if self.game_mode == 'all_agents':
                collectibles_left = len(self.land_collectibles) + len(self.water_collectibles)
            collected = self.total_collectibles - collectibles_left
            
            if self.total_collectibles > 0 and (collected / self.total_collectibles) > 0.7:
                result = "partial"
        
        # Ενημέρωση στατιστικών μόνο για επίγειο ρομπότ
        if self.game_mode == "ground":
            collectibles_found = 0
            enemies_count = 0
            
            # Υπολογισμός συλλεγμένων αντικειμένων
            if hasattr(self, 'total_collectibles') and hasattr(self, 'collectibles'):
                collectibles_found = self.total_collectibles - len(self.collectibles)
            
            # Υπολογισμός αριθμού εχθρών
            if hasattr(self, 'num_patrol') and hasattr(self, 'num_aggressive'):
                enemies_count = self.num_patrol + self.num_aggressive
            
            # Ενημέρωση στατιστικών
            update_ground_robot_stats(result, self.final_time, collectibles_found, enemies_count)
        
        # Ορισμός τελικού αποτελέσματος
        if result == "success":
            self.mission_complete = True
        elif result == "partial":
            self.mission_partial = True
        else:
            self.mission_failed = True
    
    def draw(self):
        """ Κύρια συνάρτηση σχεδιασμού. Σχεδιάζει το χάρτη, τα οχήματα, τα αντικείμενα και το UI. """
        self.screen.fill(BLACK)
        self.draw_grid()
        
        # Λεξικό με draw functions για κάθε τρόπο παιχνιδιού
        draw_funcs = {
            "ground": self.draw_ground,
            "single_drone": self.draw_single_vehicle,
            "single_aquatic": self.draw_single_vehicle,
            "coop_ground_drone": self.draw_coop_ground_drone,
            "drone_aquatic": self.draw_drone_aquatic,
            "all_agents": self.draw_all_agents
        }
        
        draw_funcs.get(self.game_mode, self.draw_ground)()
        self.draw_ui()
        pygame.display.flip()
    
    def draw_grid(self):
        """ Σχεδιάζει το πλέγμα του χάρτη. Χειρίζεται fog of war για σενάρια με drones. """
        is_fog_of_war_mode = hasattr(self, 'revealed_map')
        
        for y in range(GRID_HEIGHT_CELLS):
            for x in range(GRID_WIDTH_CELLS):
                # Σχεδιασμός σκοτεινών κελιών (fog of war)
                if is_fog_of_war_mode and not self.revealed_map[y][x]:
                    pygame.draw.rect(self.screen, (20, 20, 20), 
                                   (x * TILE_SIZE, y * TILE_SIZE, TILE_SIZE, TILE_SIZE))
                    continue
                
                # Σχεδιασμός κελιού βάσει τύπου
                tile = self.game_map[y][x]
                if tile == 1:
                    color = COLOR_OBSTACLE
                elif tile == 2:
                    color = COLOR_DIFFICULT
                elif tile == 3:
                    color = COLOR_HAZARD
                elif tile == 4:
                    color = COLOR_WATER
                else:
                    color = COLOR_FREE
                
                pygame.draw.rect(self.screen, color, 
                               (x * TILE_SIZE, y * TILE_SIZE, TILE_SIZE, TILE_SIZE))
                
                # Περίγραμμα για μη-υδάτινα κελιά
                if color != COLOR_WATER:
                    pygame.draw.rect(self.screen, BLACK, 
                                   (x * TILE_SIZE, y * TILE_SIZE, TILE_SIZE, TILE_SIZE), 1)
    
    def draw_ground(self):
        """Σχεδιάζει την οθόνη για αποστολή επίγειου ρομπότ"""
        if hasattr(self, 'agent') and self.agent.path:
            self.draw_path(self.agent.path)
        
        if hasattr(self, 'collectibles'):
            for c in self.collectibles:
                self.draw_collectible(c)
        
        if hasattr(self, 'current_target') and self.current_target:
            self.draw_goal_outline(self.current_target)
        
        if hasattr(self, 'dynamic_obstacles'):
            for obs in self.dynamic_obstacles:
                obs.draw(self.screen)
        
        if hasattr(self, 'agent'):
            self.agent.draw(self.screen)
    
    def draw_single_vehicle(self):
        """Σχεδιάζει την οθόνη για αποστολή ενός οχήματος"""
        self.draw_collectible(self.target)
        self.vehicle.draw(self.screen)
    
    def draw_coop_ground_drone(self):
        """Σχεδιάζει την οθόνη για συνεργατική αποστολή drone + επίγειο"""
        if self.phase == "ground_search" and self.agent.path:
            self.draw_path(self.agent.path)
        
        for c in self.collectibles:
            self.draw_collectible(c)
        
        if self.phase == "drone_scan":
            self.drone.draw(self.screen)
        else:
            self.agent.draw(self.screen)
    
    def draw_drone_aquatic(self):
        """Σχεδιάζει την οθόνη για αποστολή drone + υδάτινο"""
        self.draw_collectible(self.target)
        self.drone.draw(self.screen)
        self.aquatic.draw(self.screen)
    
    def draw_all_agents(self):
        """Σχεδιάζει την οθόνη για πλήρη αποστολή με όλα τα οχήματα"""
        if self.phase == "coordinated_search" and self.agent.path:
            self.draw_path(self.agent.path)
        
        for c in self.land_collectibles + self.water_collectibles:
            self.draw_collectible(c)
        
        self.agent.draw(self.screen)
        self.aquatic.draw(self.screen)
        
        if self.phase == "drone_scan":
            self.drone.draw(self.screen)
    
    def draw_path(self, path):
        """ Σχεδιάζει τη διαδρομή του ρομπότ με διαφάνεια. path: λίστα συντεταγμένων διαδρομής """
        path_surf = pygame.Surface((SCREEN_WIDTH, SCREEN_HEIGHT), pygame.SRCALPHA)
        
        for i, step in enumerate(path):
            # Μείωση διαφάνειας για τα πιο μακρινά σημεία
            alpha = max(50, 200 - i * 15)
            pygame.draw.rect(path_surf, (*COLOR_PATH[:3], alpha),
                           (step[0] * TILE_SIZE + 5, step[1] * TILE_SIZE + 5, 
                            TILE_SIZE - 10, TILE_SIZE - 10), border_radius=3)
        
        self.screen.blit(path_surf, (0, 0))
    
    def draw_collectible(self, pos):
        """Σχεδιάζει ένα αντικείμενο συλλογής"""
        pygame.draw.circle(self.screen, COLOR_COLLECTIBLE,
                         (pos[0] * TILE_SIZE + TILE_SIZE // 2, 
                          pos[1] * TILE_SIZE + TILE_SIZE // 2), TILE_SIZE // 3)
    
    def draw_goal_outline(self, pos):
        """Σχεδιάζει περίγραμμα γύρω από τον τρέχοντα στόχο"""
        pygame.draw.rect(self.screen, COLOR_GOAL_OUTLINE,
                        (pos[0] * TILE_SIZE, pos[1] * TILE_SIZE, TILE_SIZE, TILE_SIZE), 3)
    
    def draw_ui(self):
        """ Σχεδιάζει το UI (χρόνος, στόχοι, εχθροί, μηνύματα). """
        # Χρόνος αποστολής
        time_val = (pygame.time.get_ticks() - self.start_time) // 1000 if not self.final_time else self.final_time
        time_text = self.font.render(f"Time: {time_val}", True, WHITE)
        self.screen.blit(time_text, (10, 10))
        
        # Αντικείμενα συλλογής
        if hasattr(self, 'total_collectibles') and self.total_collectibles > 0:
            current_collectibles = len(getattr(self, 'collectibles', []))
            if self.game_mode == 'all_agents':
                current_collectibles = len(self.land_collectibles) + len(self.water_collectibles)
            collected = self.total_collectibles - current_collectibles
            targets_text = self.font.render(f"Targets: {collected} / {self.total_collectibles}", True, WHITE)
            self.screen.blit(targets_text, (150, 10))
        
        # Εχθροί και στατιστικά (μόνο για επίγειο σενάριο)
        if self.game_mode == "ground":
            enemies_text = self.font.render(f"Enemies: {self.num_patrol + self.num_aggressive} ({self.num_patrol}P, {self.num_aggressive}A)", True, WHITE)
            self.screen.blit(enemies_text, (SCREEN_WIDTH - 250, 10))
            
            # Εμφάνιση στατιστικών επιτυχίας
            stats = load_ground_robot_stats()
            if stats["total_missions"] > 0:
                success_text = self.font.render(f"Success Rate: {stats['success_rate']:.1f}% ({stats['successful_missions']}/{stats['total_missions']})", True, (0, 255, 0))
                self.screen.blit(success_text, (10, 40))
                
                if stats["best_time"] != float('inf'):
                    best_time_text = self.font.render(f"Best Time: {stats['best_time']}s", True, (255, 255, 0))
                    self.screen.blit(best_time_text, (10, 70))
        
        # Μήνυμα εντοπισμού στόχου
        if hasattr(self, 'target_found_message') and self.target_found_message:
            msg_surf = self.font.render(self.target_found_message, True, (0, 255, 0))
            rect = msg_surf.get_rect(center=(SCREEN_WIDTH / 2, SCREEN_HEIGHT - 20))
            self.screen.blit(msg_surf, rect)
        
        # Μήνυμα παύσης
        if self.paused:
            pause_msg = "PAUSED - Press SPACE to continue"
            pause_surf = self.font.render(pause_msg, True, (255, 255, 0))
            rect = pause_surf.get_rect(center=(SCREEN_WIDTH / 2, SCREEN_HEIGHT / 2))
            pygame.draw.rect(self.screen, BLACK, rect.inflate(20, 10))
            self.screen.blit(pause_surf, rect)
        
        # Μήνυμα αποτελέσματος αποστολής
        msg, color = ("", (0, 0, 0))
        if self.mission_complete:
            msg, color = "APOSTOLI EPITYCHIS! (R for New)", (0, 255, 0)
        elif self.mission_partial:
            msg, color = "MERIKH EPITYCHIA! (R for New)", (255, 255, 0)
        elif self.mission_failed:
            msg, color = "APOSTOLI APETYXE! (R for New)", (255, 0, 0)
        
        if msg and not self.paused:
            msg_surf = self.font.render(msg, True, color)
            rect = msg_surf.get_rect(center=(SCREEN_WIDTH / 2, SCREEN_HEIGHT / 2))
            pygame.draw.rect(self.screen, BLACK, rect.inflate(20, 10))
            self.screen.blit(msg_surf, rect)
            
            
 # Στατιστικά επίγειου ρομπότ (κάτω αριστερά)
        if self.game_mode == "ground":   
            stats = load_ground_robot_stats()
            stats_y = SCREEN_HEIGHT - 80  # Κάτω αριστερά
                   
            # Debug: εμφάνιση πάντα κάποιο μήνυμα
            debug_text = self.font.render(f"DEBUG: {stats['total_missions']} missions", True, (255, 255, 255))
            self.screen.blit(debug_text, (10, stats_y - 30))
            if stats["total_missions"] > 0:
                # Ποσοστό επιτυχίας
                success_text = self.font.render(f"Success: {stats['success_rate']:.1f}% ({stats['successful_missions']}/{stats['total_missions']})", True, (0, 255, 0))
                self.screen.blit(success_text, (10, stats_y))
                # Καλύτερος χρόνος
                if stats["best_time"] != float('inf'):
                    best_time_text = self.font.render(f"Best Time: {stats['best_time']}s", True, (255, 255, 0))
                    self.screen.blit(best_time_text, (10, stats_y + 25))
            else:
# Μήνυμα αν δεν υπάρχουν στατιστικά
                no_stats_text = self.font.render("No stats yet - Complete a mission!", True, (200, 200, 200))
                self.screen.blit(no_stats_text, (10, stats_y))


# ΜΕΝΟΥ
def main_menu():
    """ Κύριο μενού επιλογής σεναρίου αποστολής. Επιστρέφει το επιλεγμένο τρόπο παιχνιδιού ή None για έξοδο. """
    screen = pygame.display.set_mode((550, 450))
    pygame.display.set_caption("SAR - Κεντρικό Μενού")
    font = pygame.font.SysFont("Arial", 24)
    selected = 0
    
    # Επιλογές μενού
    options = [
        "1. Επίγειο Ρομπότ",
        "2. Αποστολή Drone", 
        "3. Αποστολή Υδάτινου",
        "4. Drone + Επίγειο (Cooperative)",
        "5. Drone + Υδάτινο",
        "6. Όλα Μαζί (Λίμνη)",
        "7. Στατιστικά Επίγειου Ρομπότ" 
    ]
    keys = ["ground", "single_drone", "single_aquatic", "coop_ground_drone", "drone_aquatic", "all_agents", "stats"]
    
    while True:
        screen.fill(MENU_BG_COLOR)
        
        # Τίτλος
        title = font.render("SAR - Επιλογή Σεναρίου", True, (255, 255, 0))
        screen.blit(title, title.get_rect(center=(275, 40)))
        
        # Επιλογές
        for i, opt in enumerate(options):
            color = MENU_HIGHLIGHT_COLOR if i == selected else MENU_TEXT_COLOR
            screen.blit(font.render(opt, True, color), 
                       font.render(opt, True, color).get_rect(center=(275, 100 + i * 45)))
        
        pygame.display.flip()
        
        # Χείριση events
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return None
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_DOWN:
                    selected = (selected + 1) % len(options)
                if event.key == pygame.K_UP:
                    selected = (selected - 1) % len(options)
                if event.key == pygame.K_RETURN:
                    return keys[selected]
                if event.key == pygame.K_ESCAPE:
                    return None

def pattern_menu(title):
    """ Μενού επιλογής μοτίβου αναζήτησης.
    title: τίτλος του μενού
    Επιστρέφει το επιλεγμένο μοτίβο ή None για ακύρωση. """
    screen = pygame.display.set_mode((550, 300))
    pygame.display.set_caption(title)
    font = pygame.font.SysFont("Arial", 24)
    selected = 0
    
    patterns = ["expanding_square", "sector_search", "parallel_search"]
    labels = ["Expanding Square", "Sector Search", "Parallel Track"]
    
    while True:
        screen.fill(MENU_BG_COLOR)
        
        # Τίτλος
        title_surf = font.render(title, True, (255, 255, 0))
        screen.blit(title_surf, title_surf.get_rect(center=(275, 40)))
        
        # Επιλογές
        for i, label in enumerate(labels):
            color = MENU_HIGHLIGHT_COLOR if i == selected else MENU_TEXT_COLOR
            screen.blit(font.render(label, True, color), 
                       font.render(label, True, color).get_rect(center=(275, 100 + i * 40)))
        
        pygame.display.flip()
        
        # Χείριση events
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return "quit"
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_DOWN:
                    selected = (selected + 1) % len(labels)
                if event.key == pygame.K_UP:
                    selected = (selected - 1) % len(labels)
                if event.key == pygame.K_RETURN:
                    return patterns[selected]
                if event.key == pygame.K_ESCAPE:
                    return None

def enemy_selection_menu():
    """ Μενού επιλογής αριθμού εχθρών για επίγειο σενάριο. 
    Επιστρέφει tuple (num_patrol, num_aggressive) ή None για ακύρωση. """
    screen = pygame.display.set_mode((550, 300))
    pygame.display.set_caption("SAR - Ρύθμιση Εχθρών")
    font = pygame.font.SysFont("Arial", 24)
    clock = pygame.time.Clock()
    
    # Αρχικές τιμές
    num_patrol, num_aggressive = 3, 2
    min_val, max_patrol, max_aggr = 0, 5, 3
    selected_option = 0
    
    while True:
        screen.fill(MENU_BG_COLOR)
        
        # Τίτλος
        title = font.render("Ρύθμιση Εχθρών", True, (255, 255, 0))
        screen.blit(title, title.get_rect(center=(275, 40)))
        
        # Εχθροί περιπολίας
        color_p = MENU_HIGHLIGHT_COLOR if selected_option == 0 else MENU_TEXT_COLOR
        patrol_txt = font.render(f"Εχθροί Περιπολίας: < {num_patrol} >", True, color_p)
        screen.blit(patrol_txt, patrol_txt.get_rect(center=(275, 120)))
        
        # Επιθετικοί εχθροί
        color_a = MENU_HIGHLIGHT_COLOR if selected_option == 1 else MENU_TEXT_COLOR
        aggr_txt = font.render(f"Επιθετικοί Εχθροί: < {num_aggressive} >", True, color_a)
        screen.blit(aggr_txt, aggr_txt.get_rect(center=(275, 170)))
        
        # Οδηγίες
        instr = font.render("Enter: Έναρξη, Esc: Πίσω", True, (200, 200, 200))
        screen.blit(instr, instr.get_rect(center=(275, 250)))
        
        pygame.display.flip()
        
        # Χείριση events
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return None
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    return None
                if event.key == pygame.K_RETURN:
                    return num_patrol, num_aggressive
                if event.key in [pygame.K_UP, pygame.K_DOWN]:
                    selected_option = 1 - selected_option
                if event.key == pygame.K_LEFT:
                    if selected_option == 0:
                        num_patrol = max(min_val, num_patrol - 1)
                    else:
                        num_aggressive = max(min_val, num_aggressive - 1)
                if event.key == pygame.K_RIGHT:
                    if selected_option == 0:
                        num_patrol = min(max_patrol, num_patrol + 1)
                    else:
                        num_aggressive = min(max_aggr, num_aggressive + 1)
        
        clock.tick(30)

def show_ground_robot_stats():
    """ Εμφανίζει τα στατιστικά του επίγειου ρομπότ σε ξεχωριστό παράθυρο. """
    screen = pygame.display.set_mode((600, 500))
    pygame.display.set_caption("SAR - Στατιστικά Επίγειου Ρομπότ")
    font_large = pygame.font.SysFont("Arial", 28)
    font_medium = pygame.font.SysFont("Arial", 20)
    font_small = pygame.font.SysFont("Arial", 16)
    clock = pygame.time.Clock()
    
    # Φόρτωση στατιστικών μία φορά, πριν τον βρόχο.
    stats = load_ground_robot_stats()
    
    while True:
        screen.fill(MENU_BG_COLOR)
        
        # Τίτλος
        title = font_large.render("Στατιστικά Επίγειου Ρομπότ", True, (255, 255, 0))
        screen.blit(title, title.get_rect(center=(300, 40)))
        
        y_pos = 100
        
        # Βασικά στατιστικά
        if stats["total_missions"] > 0:
            # Συνολικές αποστολές
            total_text = font_medium.render(f"Συνολικές Αποστολές: {stats['total_missions']}", True, MENU_TEXT_COLOR)
            screen.blit(total_text, (50, y_pos))
            y_pos += 40
            
            # Επιτυχημένες αποστολές
            success_text = font_medium.render(f"Επιτυχημένες: {stats['successful_missions']}", True, (0, 255, 0))
            screen.blit(success_text, (50, y_pos))
            y_pos += 40
            
            # Αποτυχημένες αποστολές
            failed_text = font_medium.render(f"Αποτυχημένες: {stats['failed_missions']}", True, (255, 0, 0))
            screen.blit(failed_text, (50, y_pos))
            y_pos += 40
            
            # Μερικές επιτυχίες
            partial_text = font_medium.render(f"Μερικές Επιτυχίες: {stats['partial_missions']}", True, (255, 255, 0))
            screen.blit(partial_text, (50, y_pos))
            y_pos += 40
            
            # Ποσοστό επιτυχίας
            rate_text = font_medium.render(f"Ποσοστό Επιτυχίας: {stats['success_rate']:.1f}%", True, (0, 255, 255))
            screen.blit(rate_text, (50, y_pos))
            y_pos += 40
            
            # Μέσος χρόνος
            avg_time_text = font_medium.render(f"Μέσος Χρόνος: {stats['average_time']:.1f} δευτερόλεπτα", True, MENU_TEXT_COLOR)
            screen.blit(avg_time_text, (50, y_pos))
            y_pos += 40
            
            # Καλύτερος χρόνος
            if stats["best_time"] != float('inf'):
                best_time_text = font_medium.render(f"Καλύτερος Χρόνος: {stats['best_time']} δευτερόλεπτα", True, (255, 215, 0))
                screen.blit(best_time_text, (50, y_pos))
                y_pos += 40
            
            # Συλλεγμένα αντικείμενα
            collectibles_text = font_medium.render(f"Συλλεγμένα Αντικείμενα: {stats['total_collectibles_found']}", True, (255, 165, 0))
            screen.blit(collectibles_text, (50, y_pos))
            y_pos += 40
            
            # Εχθροί που αντιμετώπισε
            enemies_text = font_medium.render(f"Εχθροί που Αντιμετώπισε: {stats['total_enemies_encountered']}", True, (255, 100, 100))
            screen.blit(enemies_text, (50, y_pos))
        else:
            # Μήνυμα αν δεν υπάρχουν στατιστικά
            no_stats_text = font_medium.render("Δεν υπάρχουν στατιστικά ακόμα.", True, MENU_TEXT_COLOR)
            screen.blit(no_stats_text, no_stats_text.get_rect(center=(300, 200)))
            no_stats_text2 = font_medium.render("Παίξτε μερικές αποστολές επίγειου ρομπότ!", True, MENU_TEXT_COLOR)
            screen.blit(no_stats_text2, no_stats_text2.get_rect(center=(300, 240)))
            
        # Οδηγίες
        instr_text = font_small.render("Πατήστε ESC για επιστροφή στο μενού", True, (200, 200, 200))
        screen.blit(instr_text, instr_text.get_rect(center=(300, 450)))
        
        pygame.display.flip()
        
        # Χείριση events
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return "quit"
            if event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                return None
        
        clock.tick(30)

# ΚΥΡΙΟ ΠΡΟΓΡΑΜΜΑ

def main():
    """ Κύρια συνάρτηση του προγράμματος. """
    pygame.init()
    
    while True:
        # Επιλογή τρόπου παιχνιδιού
        game_mode = main_menu()
        if game_mode is None:
            break
        
        # Εμφάνιση στατιστικών αν επιλέχθηκε
        if game_mode == "stats":
            result = show_ground_robot_stats()
            if result == "quit":
                break
            continue
        
        # Ρύθμιση εχθρών για επίγειο σενάριο
        params = {"game_mode": game_mode}
        if game_mode == "ground":
            enemy_settings = enemy_selection_menu()
            if enemy_settings is None:
                continue
            params["num_patrol"], params["num_aggressive"] = enemy_settings
        
        # Επιλογή μοτίβου αναζήτησης για drone
        if game_mode in ["single_drone", "coop_ground_drone", "all_agents"]:
            p = pattern_menu("Επιλογή Μοτίβου Drone")
            if p in [None, "quit"]:
                continue
            params["pattern"] = p
        
        # Επιλογή μοτίβου για υδάτινο όχημα
        elif game_mode == "single_aquatic":
            p = pattern_menu("Επιλογή Μοτίβου Υδάτινου")
            if p in [None, "quit"]:
                continue
            params["pattern"] = p
        
        # Επιλογή μοτίβων για drone + υδάτινο
        elif game_mode == "drone_aquatic":
            dp = pattern_menu("Επιλογή Μοτίβου Drone")
            if dp in [None, "quit"]:
                continue
            params["drone_pattern"] = dp
            
            ap = pattern_menu("Επιλογή Μοτίβου Υδάτινου")
            if ap in [None, "quit"]:
                continue
            params["aquatic_pattern"] = ap
        
        # Εκκίνηση παιχνιδιού
        game = Game(**params)
        if not game.run():
            break
            
    pygame.quit()

if __name__ == '__main__':
    main()