import copy
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from sensor_msgs.msg import Image
import scipy.ndimage as scipy
import cv2 as cv
import numpy as np
from cv_bridge import CvBridge
from rcl_interfaces.msg import SetParametersResult, Parameter

SCORE_WIN = 1
SCORE_DRAW = 0
SCORE_LOSS = -1


class ImageScannerService(Node):
    def __init__(self):
        super().__init__("tictactoe")
        self.camera_stream_image: cv.Mat
        self.tictactoe_map = [
            ['', '', ''],
            ['', '', ''],
            ['', '', '']
        ]

        self.declare_parameter("xORo", 'X')
        self.declare_parameter("first_turn", False)

        self.my_sign = self.get_parameter("xORo").value
        self.add_on_set_parameters_callback(self.on_parameters_set_callback)
        self.marked_image = None
        self.threshold_image = None

        self.service = self.create_service(Trigger, "image_scan", self.scan_callback)
        self.image_publisher = self.create_publisher(Image, "marked_image", 1)
        self.threshold_publisher = self.create_publisher(Image, "threshold", 1)

        self.camera_subscriber = self.create_subscription(Image, "image", self.camera_stream_callback, 1)

        self.timer = self.create_timer(1. / 10., self.timer_callback)


    def camera_stream_callback(self, msg: Image):
        self.camera_stream_image = CvBridge().imgmsg_to_cv2(msg)
        self.camera_stream_image = cv.rotate(self.camera_stream_image, cv.ROTATE_90_COUNTERCLOCKWISE)
        cv.cvtColor(self.camera_stream_image, cv.COLOR_BGR2RGB, self.camera_stream_image)
        if self.my_sign == 'X':
            self.threshold_image = cv.inRange(self.camera_stream_image, (5, 10, 60), (60, 150, 255))
        elif self.my_sign == 'O':
            self.threshold_image = cv.inRange(self.camera_stream_image, (150, 10, 10), (255, 150, 80))


    def on_parameters_set_callback(self, parameters: list[Parameter]):
        for parameter in parameters:
            if parameter.name == "xORo":
                if parameter.value != 'O' and parameter.value != 'X':
                    print("Invalid parameter")
                    return SetParametersResult(successful=False)
                self.my_sign = parameter.value

        return SetParametersResult(successful=True)
    
    def timer_callback(self):
        if self.threshold_image is not None:
            self.threshold_publisher.publish(CvBridge().cv2_to_imgmsg(self.threshold_image, "mono8"))

        if self.marked_image is not None:
            self.image_publisher.publish(CvBridge().cv2_to_imgmsg(self.marked_image, "rgb8"))


    def is_position_available(self, i, j) -> bool:
        tictactoe_index = ((i * 3) // self.camera_stream_image.shape[0], (j * 3) // self.camera_stream_image.shape[1])
        return self.tictactoe_map[tictactoe_index[0]][tictactoe_index[1]] == ''


    def find_x(self):
        self.marked_image = self.camera_stream_image[:]
        filter = np.array([
            [1/13,0,0,0,0,0,1/13],
            [0,1/13,0,0,0,1/13,0],
            [0,0,1/13,0,1/13,0,0],
            [0,0,0,1/13,0,0,0],
            [0,0,1/13,0,1/13,0,0],
            [0,1/13,0,0,0,1/13,0],
            [1/13,0,0,0,0,0,1/13],
        ])

        res = scipy.convolve(self.threshold_image, filter)
        max = 0
        index = (-1, -1)
        for i, row in enumerate(res):
            for j, column in enumerate(row):
                if column > max and self.is_position_available(i, j):
                    max = column
                    index = (i, j)

        tictactoe_index = ((index[0] * 3) // self.camera_stream_image.shape[0], (index[1] * 3) // self.camera_stream_image.shape[1])
        self.tictactoe_map[tictactoe_index[0]][tictactoe_index[1]] = 'X'
        top_left, bottom_right = self.frame(self.threshold_image, *index)
        cv.rectangle(self.marked_image, top_left[::-1], bottom_right[::-1], (0, 255, 0), 2)

    def find_o(self):
        self.marked_image = self.camera_stream_image[:]
        filter = np.ones((5, 5))
        filter /= filter.shape[0] * filter.shape[1]

        res = scipy.convolve(self.threshold_image, filter)
        max = 0
        index = (-1, -1)
        for i, row in enumerate(res):
            for j, column in enumerate(row):
                if column > max and self.is_position_available(i, j):
                    max = column
                    index = (i, j)

        tictactoe_index = ((index[0] * 3) // self.camera_stream_image.shape[0], (index[1] * 3) // self.camera_stream_image.shape[1])
        self.tictactoe_map[tictactoe_index[0]][tictactoe_index[1]] = 'O'
        top_left, bottom_right = self.frame(self.threshold_image, *index)
        cv.rectangle(self.marked_image, top_left[::-1], bottom_right[::-1], (255, 255, 0), 2)

    
    def frame(self, image, row, column):
        """
        Returns top left and bottom right coordinates of a frame
        """
        up_step = 1
        down_step = 1
        left_step = 1
        right_step = 1

        loop = True
        while loop:
            loop = False
            if row-up_step > 0 and column-left_step > 0 and column+right_step+1 < image.shape[1] and np.sum(image[row-up_step, column-left_step:column+right_step+1] != 0):
                up_step += 5
                loop = True

            if row+down_step < image.shape[0] and column-left_step > 0 and column+right_step+1 < image.shape[1] and np.sum(image[row+down_step, column-left_step:column+right_step+1] != 0):
                down_step += 5
                loop = True

            if row-up_step > 0 and row+down_step+1 < image.shape[0] and column-left_step > 0 and np.sum(image[row-up_step: row+down_step+1, column-left_step] != 0):
                left_step += 5
                loop = True

            if row-up_step > 0 and row+down_step+1 < image.shape[0] and column+right_step < image.shape[1] and np.sum(image[row-up_step: row+down_step+1, column+right_step] != 0):
                right_step += 5
                loop = True
        
        return_coords = (row-up_step, column-left_step), (row+down_step, column+right_step)
        return return_coords
    
    
    def scan_callback(self, request, response):
        if self.my_sign == 'X':
            self.find_x()
        else:
            self.find_o()
        
    
        message = "Game on..."
        if self.check_for_win(self.tictactoe_map, "X"):
            self.tictactoe_map = [["", "", ""], ["", "", ""], ["", "", ""]]
            message = "You won!"
        elif self.check_for_win(self.tictactoe_map, "O"):
            self.tictactoe_map = [["", "", ""], ["", "", ""], ["", "", ""]]
            message = "CPU won!"
        elif self.check_for_tie(self.tictactoe_map):
            self.tictactoe_map = [["", "", ""], ["", "", ""], ["", "", ""]]
            message = "Draw!"
        
        if message == "Game on...":
            best_move = self.get_best_move()
            self.tictactoe_map[best_move[0]][best_move[1]] = 'O'
            if self.check_for_win(self.tictactoe_map, "X"):
                self.tictactoe_map = [["", "", ""], ["", "", ""], ["", "", ""]]
                message = "You won!"
            elif self.check_for_win(self.tictactoe_map, "O"):
                self.tictactoe_map = [["", "", ""], ["", "", ""], ["", "", ""]]
                message = "CPU won!"
            elif self.check_for_tie(self.tictactoe_map):
                self.tictactoe_map = [["", "", ""], ["", "", ""], ["", "", ""]]
                message = "Draw!"
        

        response.message = str(self.tictactoe_map) + ", " + message
        response.success = True
        return response


    def minimax(self, board, depth, is_maximizing):
        # is_maximizing je isto sto i IS_AI
        if self.check_for_win(board, "X"):
            return SCORE_WIN
            #self.get_logger().info("You won!")
            #self.destroy_node()
        elif self.check_for_win(board, "O"):
            return SCORE_LOSS
            #self.get_logger().info("You lost!")
            #self.destroy_node()
        elif depth <= 0 or self.check_for_tie(board):
            return SCORE_DRAW
            #self.get_logger().info("Draw!")
            #self.destroy_node()

        if is_maximizing:
            # We are simulating this round as AI
            best_score = SCORE_LOSS
            symbol = "O"
        else:
            # We are simulating this round as Player
            best_score = SCORE_WIN
            symbol = "X"

        for i in range(3):
            for j in range(3):
                if board[i][j] == "":
                    new_board = copy.deepcopy(board)
                    new_board[i][j] = symbol
                    # print("")
                    # print("")
                    # print_board(new_board)
                    score = self.minimax(new_board, depth - 1, not is_maximizing)

                    if is_maximizing:
                        best_score = max(best_score, score)
                    else:
                        best_score = min(best_score, score)

        return best_score


    def get_best_move(self):
        board = self.tictactoe_map
        best_move = (0, 0)  # Set to first field regardless of if it is available
        best_score = SCORE_LOSS  # Set to worst case scenario
        depth = 9 - self.get_move_counter(board)

        # We are on our first AI move and player has not put it into center. Statistically,
        # center is the best option, so always do that in that case.
        if depth == 8 and board[1][1] == "":
            return (1, 1)

        for i in range(3):
            for j in range(3):
                if board[i][j] == "":
                    # We are on our first AI move and player has put it into center, just
                    # skip all other calculations since it does not really matter at this
                    # point where we put it.
                    if depth == 8:
                        return (i, j)

                    board[i][j] = "O"
                    value = self.minimax(board, depth - 1, False)
                    board[i][j] = ""

                    if value > best_score:
                        best_score = value
                        best_move = (i, j)

        return best_move


    def get_move_counter(self, board):
        count = 0
        for row in board:
            for el in row:
                if el != "":
                    count += 1
        return count


    def check_for_win(self, board, symbol):
        # Checking the rows
        for i in range(3):
            if board[i][0] == symbol and board[i][1] == symbol and board[i][2] == symbol:
                return True
        # Checking the columns
        for i in range(3):
            if board[0][i] == symbol and board[1][i] == symbol and board[2][i] == symbol:
                return True
        # Checking the diagonals
        if board[0][0] == symbol and board[1][1] == symbol and board[2][2] == symbol:
            return True
        if board[0][2] == symbol and board[1][1] == symbol and board[2][0] == symbol:
            return True
        return False


    def check_for_tie(self, board):
        for i in range(3):
            for j in range(3):
                if board[i][j] == "":
                    return False
        return True

def main():
    rclpy.init()
    image_scanner = ImageScannerService()
    rclpy.spin(image_scanner)
    image_scanner.destroy_node()
    rclpy.shutdown()
