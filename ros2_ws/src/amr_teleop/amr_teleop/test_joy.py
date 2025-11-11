from tkinter import Canvas, Tk

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from sensor_msgs.msg import Joy


class JoyButton:
    def __init__(self, parent_canvas, left_space, v_space, height, i):
        self.parent_canvas = parent_canvas
        lf = left_space + 15
        t = v_space + (height + v_space)*i
        self.parent_canvas.create_text(left_space, t+height/2, text=str(i))
        self.circle_obj = self.parent_canvas.create_oval(lf,
                                                        t,
                                                        lf + height,
                                                        t + height,
                                                        width=2, fill='white')

    def update_value(self, value):
        if (value > 0):
            self.parent_canvas.itemconfigure(self.circle_obj, fill='#FF0000')
        else:
            self.parent_canvas.itemconfigure(self.circle_obj, fill='#FFFFFF')


class JoyAxis:
    def __init__(self, parent_canvas, left_space, v_space, height, width, i):
        self.parent_canvas = parent_canvas
        self.left_space = left_space
        self.v_space = v_space
        self.height = height
        self.width = width
        self.i = i

        lf = left_space + 60
        t = v_space + (height + v_space)*i
        self.parent_canvas.create_text(left_space+50, t+height/2, text=str(i))

        self.fill_obj = self.parent_canvas.create_rectangle(lf,
                                                            t,
                                                            lf + width,
                                                            t + height,
                                                            width=0, fill='green')
        self.outline_obj = self.parent_canvas.create_rectangle(lf,
                                                                t,
                                                                lf + width,
                                                                t + height,
                                                                width=2, outline='black')

        self.val_txt = self.parent_canvas.create_text(left_space+60 + width + 30,
                                                    t+height/2,
                                                    text=str(i))

    def update_value(self, value):
        lf = self.left_space + 60
        t = self.v_space + (self.height + self.v_space)*self.i

        ww = self.width * (value + 1)/2

        self.parent_canvas.coords(self.fill_obj, lf, t, lf+ww, t+self.height)
        self.parent_canvas.itemconfigure(self.val_txt, text=str(f'{value:.3f}'))


class JoyTester(Node):

    def __init__(self):
        super().__init__('test_joy')
        self.get_logger().info('Testing Joystick...')

        self.subscription = self.create_subscription(Joy, 'joy', self.joy_callback, 5)
        self.subscription  # prevent unused variable warning

        self.buttons = []
        self.axes = []
        self.initialised = False

        self.tk = Tk()

        self.canvas = Canvas(self.tk, width=800, height=480, bg='white')
        self.tk.title('Joystick Test')
        self.tk.geometry('800x480+0+0')
        self.canvas.pack(anchor='nw')

        # Create waiting message
        self.waiting_text = self.canvas.create_text(
            400, 240,
            text='Waiting for joystick data on /joy topic...\n\n'
                'Please ensure:\n'
                '1. Your joystick is connected\n'
                '2. joy_node is running (ros2 run joy joy_node)',
            font=('Arial', 14),
            justify='center',
            fill='gray'
        )

        self.tk.update()

    def joy_callback(self, joy_msg):

        left_space = 10
        height = 25
        width = 80
        v_space = 5

        # Handle first receive

        if not self.initialised:
            # Remove waiting message
            self.canvas.delete(self.waiting_text)
            
            # Add headers for buttons and axes
            self.canvas.create_text(50, 10, text='BUTTONS', font=('Arial', 12, 'bold'))
            self.canvas.create_text(400, 10, text='AXES', font=('Arial', 12, 'bold'))
            
            # Create buttons on the left side
            for i, val in enumerate(joy_msg.buttons):
                self.buttons.append(JoyButton(self.canvas, left_space, v_space + 30, height, i))

            # Create axes on the right side
            for i, val in enumerate(joy_msg.axes):
                self.axes.append(JoyAxis(self.canvas, 300, v_space + 30, height, width, i))

            self.initialised = True
            self.get_logger().info(f'Joystick detected: {len(joy_msg.buttons)} buttons, {len(joy_msg.axes)} axes')

        # Update Values

        for i, val in enumerate(joy_msg.buttons):
            self.buttons[i].update_value(val)

        for i, val in enumerate(joy_msg.axes):
            self.axes[i].update_value(val)

        # Redraw
        self.tk.update()

        return


def main(args=None):
    rclpy.init(args=args)
    joy_tester = JoyTester()

    try:
        rclpy.spin(joy_tester)
    except KeyboardInterrupt:
        print('Received keyboard interrupt!')
    except ExternalShutdownException:
        print('Received external shutdown request!')

    print('Exiting...')

    joy_tester.destroy_node()
    rclpy.try_shutdown()
