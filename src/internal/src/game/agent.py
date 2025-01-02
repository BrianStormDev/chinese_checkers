#!/usr/bin/env python
from point import Point
from player import Player
from typing import List, Tuple, Dict
import numpy as np
import time

class Agent:
    def __init__(self, player: Player, game, enemy: Player):
        self.player = player
        self.enemy = enemy
        self.game = game
        self.best_move = None
        self.transposition_table = {} #: Dict[str, Tuple[float, int]]

    def order_moves(self, moves: List[Tuple[Point, str]]) -> List[Tuple[Point, str]]:
        """
        Orders moves based on a combination of jump prioritization and height change.
        """
        def move_value(move):
            move_code = move[1]
            return self.game.calculate_height_change_from_move_code(move_code)
        
        return sorted(moves, key=move_value, reverse=True)

    def minimax(self, depth: int, is_maximizing: bool, alpha: float, beta: float) -> float:
        board_hash = self.get_board_hash()
        if board_hash in self.transposition_table:
            stored_value, stored_depth = self.transposition_table[board_hash]
            if stored_depth >= depth:
                return stored_value

        if depth == 0 or self.game.is_game_over():
            return self.evaluate()

        if is_maximizing:
            max_eval = float('-inf')
            moves = self.game.valid_player_moves(self.player)
            moves = self.order_moves(moves)
            for move in moves:
                self.game.make_move(move)
                eval = self.minimax(depth - 1, False, alpha, beta)
                self.game.undo_move()
                if eval > max_eval:
                    max_eval = eval
                    if depth == self.initial_depth:
                        self.best_move = move
                alpha = max(alpha, eval)
                if beta <= alpha:
                    break
            self.transposition_table[board_hash] = (max_eval, depth)
            return max_eval
        else:
            min_eval = float('inf')
            moves = self.game.valid_player_moves(self.enemy)
            moves = self.order_moves(moves)
            for move in moves:
                self.game.make_move(move)
                eval = self.minimax(depth - 1, True, alpha, beta)
                self.game.undo_move()
                min_eval = min(min_eval, eval)
                beta = min(beta, eval)
                if beta <= alpha:
                    break
            self.transposition_table[board_hash] = (min_eval, depth)
            return min_eval
    
    def evaluate(self) -> float:
        """
        Evaluate board state with progress toward winning as the primary focus.
        """
        score = 0
        end_zone_pegs = 0  # Count of pegs securely in the end zone
        stranded_pegs_score = 0

        for peg in self.player.current_pegs:
            if self.game.in_end_zone(self.player, peg.position):
                end_zone_pegs += 1  # Count securely placed pegs
            else:
                # Penalize pegs still outside the end zone based on distance to the closest goal
                closest_goal_dist = min(
                    abs(peg.position.x - goal.x) + abs(peg.position.y - goal.y)
                    for goal in self.game.get_opposite_player(self.player).end_zone_points
                )
                stranded_pegs_score -= closest_goal_dist * 10  # Stronger penalty for stranded pegs

        # Reward having more pegs securely in the end zone
        score += end_zone_pegs * 500  # Higher weight for end zone pegs
        
        # Strong reward for completing the win condition
        if end_zone_pegs == len(self.player.current_pegs):
            score += 10000  # Huge bonus for achieving the win condition

        # Add stranded pegs score (negative impact)
        score += stranded_pegs_score

        # Enemy evaluation (opposite of player's strategy)
        enemy_score = 0
        for peg in self.enemy.current_pegs:
            if self.game.in_end_zone(self.enemy, peg.position):
                enemy_score += 250  # Reward for enemy's end zone pegs
            else:
                closest_goal_dist = min(
                    abs(peg.position.x - goal.x) + abs(peg.position.y - goal.y)
                    for goal in self.game.get_opposite_player(self.enemy).end_zone_points
                )
                enemy_score -= closest_goal_dist * 5  # Penalize enemy's stranded pegs less

        # Subtract enemy's score from player's score
        return score - enemy_score

    def get_board_hash(self) -> str:
        """
        Converts a board that is more easily interpreted by the AI
        The pegs are on the board are mapped to values between -1 and 6 to represent if those spots are 
        unreachable, empty, or belong to a player.
        """
        new_board = np.ndarray((self.game.x_dim, self.game.y_dim))

        def peg_to_color(peg):
            return self.game.color_to_number[peg.color]
        new_board = np.vectorize(peg_to_color)(self.game.board)
        
        return str(new_board.tobytes())

    def get_best_move(self, max_time) -> Tuple[Point, str, Point]:
        self.best_move = None
        self.initial_depth = 2
        start_time = time.time()

        while time.time() - start_time < max_time:
            if self.game.is_midgame():  
                max_depth = 3
            elif self.game.is_endgame():
                max_depth = 5
            else:
                max_depth = 4

            if self.initial_depth > max_depth:
                break

            self.minimax(self.initial_depth, True, float('-inf'), float('inf'))
            self.initial_depth += 1

        return self.best_move