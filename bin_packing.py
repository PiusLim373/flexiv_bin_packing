import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import tkinter as tk
import random

ROTATION = ["LWH", "WLH"]
STRIDE = ["x", "y", "z"]


class Box:
    def __init__(self, length, width, height, name="box"):
        self.length = length
        self.width = width
        self.height = height
        self.name = name
        self.items_in_box = []

    # intersection check
    def has_intersection(self, inter_check_coor, inter_check_dim, placing_coor, item_dim):
        inter_check_x, inter_check_y, inter_check_z = inter_check_coor
        inter_check_length, inter_check_width, inter_check_height = inter_check_dim
        placing_x, placing_y, placing_z = placing_coor
        item_length, item_width, item_height = item_dim

        x_intersect = (inter_check_x < placing_x + item_length) and (inter_check_x + inter_check_length > placing_x)
        y_intersect = (inter_check_y < placing_y + item_width) and (inter_check_y + inter_check_width > placing_y)
        z_intersect = (inter_check_z < placing_z + item_height) and (inter_check_z + inter_check_height > placing_z)
        return x_intersect and y_intersect and z_intersect

    # box limit check
    def has_exceeded_box_limit(self, placing_coor, item_dimension):
        if (
            (placing_coor[0] + item_dimension[0]) > self.length
            or (placing_coor[1] + item_dimension[1]) > self.width
            or (placing_coor[2] + item_dimension[2]) > self.height
        ):
            return True
        return False

    # gravity fix
    def get_all_z_placing_coor(self, data):
        x, y, z = data
        for i in range(z + 1):
            yield (x, y, i)

    def reset(self):
        self.items_in_box = []

    # main packing function
    def put_item(self, item, consider_gravity):
        # box still empty, just put at origin
        if len(self.items_in_box) == 0:
            for rotation_type in item.available_rotations:
                item.set_rotation_type(rotation_type)
                if self.has_exceeded_box_limit((0, 0, 0), item.get_rotated_dimension()):
                    continue
                self.items_in_box.append(item)
                return (True, (0, 0, 0))
            return (False, (0, 0, 0))
        else:
            # try offset along diff axis (x, y, and z)
            for stride_dir in STRIDE:
                for item_in_box in self.items_in_box:
                    item_in_box_dim = item_in_box.get_rotated_dimension()
                    item_in_box_coor = item_in_box.coor

                    # try diff rotation, starting with the one with largest surface area
                    for rotation_type in item.available_rotations:
                        item.set_rotation_type(rotation_type)
                        item_dim = item.get_rotated_dimension()
                        if stride_dir == "x":
                            placing_coor = (
                                item_in_box_coor[0] + item_in_box_dim[0],
                                item_in_box_coor[1],
                                item_in_box_coor[2],
                            )
                        elif stride_dir == "y":
                            placing_coor = (
                                item_in_box_coor[0],
                                item_in_box_coor[1] + item_in_box_dim[1],
                                item_in_box_coor[2],
                            )
                        elif stride_dir == "z":
                            placing_coor = (
                                item_in_box_coor[0],
                                item_in_box_coor[1],
                                item_in_box_coor[2] + item_in_box_dim[2],
                            )

                        # if gravity check is enabled, generate all coordinate to try
                        if consider_gravity:
                            z_placing_coor_to_try = self.get_all_z_placing_coor(placing_coor)
                        else:
                            z_placing_coor_to_try = [placing_coor]

                        for placing_coor_with_z in z_placing_coor_to_try:
                            # check if exceeded box limit
                            if self.has_exceeded_box_limit(placing_coor_with_z, item_dim):
                                continue
                            fit = True

                            # check for intersection
                            for item_in_box_inter_check in self.items_in_box:
                                inter_check_coor = item_in_box_inter_check.coor
                                inter_check_dim = item_in_box_inter_check.get_rotated_dimension()
                                if self.has_intersection(
                                    inter_check_coor, inter_check_dim, placing_coor_with_z, item_dim
                                ):
                                    fit = False

                            # update coordinate if all check pass, item can fit
                            if fit:
                                self.items_in_box.append(item)
                                return (True, placing_coor_with_z)

            # all options exhausted, return cant fit
            return (False, (0, 0, 0))


class Item:
    def __init__(self, length, width, height, name="item", colour=None):
        self.length = length
        self.width = width
        self.height = height
        self.name = name
        self.coor = None
        self.rotation_type = "LWH"
        self.available_rotations = ROTATION
        self.successfully_boxed = False
        if colour:
            self.colour = colour
        else:
            self.colour = [random.random() for _ in range(3)]

    def get_volume(self):
        return self.length * self.width * self.height

    def get_top_surface(self):
        length, width, _ = self.get_rotated_dimension()
        return length * width

    def print_item(self):
        print(
            f"[Item] name: {self.name}, size: {(self.length, self.width, self.height)}, volume: {self.get_volume()}, coor: {self.coor}"
        )

    # set the rotation config
    def set_rotation_type(self, rotation_type):
        self.rotation_type = rotation_type

    # get the dimension, affected by the curent rotation config
    def get_rotated_dimension(self):
        if self.rotation_type == "LWH":
            return (self.length, self.width, self.height)
        elif self.rotation_type == "WLH":
            return (self.width, self.length, self.height)
        elif self.rotation_type == "LHW":
            return (self.length, self.height, self.width)
        elif self.rotation_type == "WHL":
            return (self.width, self.height, self.length)
        elif self.rotation_type == "HWL":
            return (self.height, self.width, self.length)
        elif self.rotation_type == "HLW":
            return (self.height, self.length, self.width)
        else:
            pass

    def reset(self):
        self.rotation_type = "LWH"
        self.available_rotations = ROTATION
        self.successfully_boxed = False
        self.coor = None

    def update_successful_boxed(self, coor):
        self.successfully_boxed = True
        self.coor = coor
        print(f"[Item] item name {self.name} successfully boxed, at coor: {self.coor}")


class Packer:
    def __init__(self):
        self.box = None
        self.item_to_pack = []
        self.fig = None
        self.ax = None
        self.packing_detail_text_component = None
        self.fig_by_step = None
        self.ax_by_step = None
        self.counter_by_step = -1

    def add_box(self, box):
        self.box = box

    def add_item(self, item):
        self.item_to_pack.append(item)

    def reset(self):
        self.box = None
        self.item_to_pack = []
        self.fig = None
        self.ax = None
        self.packing_detail_text_component = None
        self.fig_by_step = None
        self.ax_by_step = None
        self.counter_by_step = -1

    def reset_item(self):
        self.item_to_pack = []

    def reset_box(self):
        self.box = None

    # most other layer of packing function, called by ui
    def pack(self, consider_stability=False, consider_gravity=False, enable_3d_rotation=False):
        global ROTATION
        print(
            f"called pack consider_stability: {consider_stability}, consider_gravity:{consider_gravity}, enable_3d_rotation: {enable_3d_rotation}"
        )
        if enable_3d_rotation:
            ROTATION = ["LWH", "WLH", "LHW", "WHL", "HWL", "HLW"]
        else:
            ROTATION = ["LWH", "WLH"]

        # reset the box and items
        self.box.reset()
        for item in self.item_to_pack:
            item.reset()
            item.print_item()

        if consider_stability:
            # if stability check and full rotation is enabled, check surface area for all rotation config and sort in decending order,
            # also set the item to be in the largest surface area rotation config
            if enable_3d_rotation:
                for item in self.item_to_pack:
                    surface_area_rot_dict = {}
                    for rotation_type in ROTATION:
                        item.set_rotation_type(rotation_type)
                        surface_area = item.get_top_surface()
                        surface_area_rot_dict[rotation_type] = surface_area
                    print(surface_area_rot_dict)
                    item.available_rotations = sorted(
                        surface_area_rot_dict, key=lambda k: surface_area_rot_dict[k], reverse=True
                    )
                    item.set_rotation_type(item.available_rotations[0])
            # determine the packing order, based on surface area, biggest first
            self.item_to_pack.sort(key=lambda item: item.get_top_surface(), reverse=True)

        # determine the packing order, based on volume, biggest first
        else:
            self.item_to_pack.sort(key=lambda item: item.get_volume(), reverse=True)

        for item in self.item_to_pack:
            # call to pack item in box, if packing is successful, update the item's coordinate
            success, coor = self.box.put_item(item, consider_gravity)
            if success:
                item.update_successful_boxed(coor)

        # post packing
        print(f"[Packer] Successfully packed {len(self.box.items_in_box)} / {len(self.item_to_pack)} items")
        # sort the packed item in bottom-up manner, useful for robot integration later on
        self.box.items_in_box = sorted(
            self.box.items_in_box, key=lambda item: (item.coor[2], item.coor[1], item.coor[0])
        )
        for item in self.box.items_in_box:
            print(
                f"[Packer] {item.name} is at {item.coor} with rotation config: {item.rotation_type}, rotated dim: {item.get_rotated_dimension()}"
            )
        # display in matplotlib 3d
        self.visualization()

    def visualization(self):
        if self.fig is None or self.ax is None:
            self.fig = plt.figure()
            self.ax = self.fig.add_subplot(111, projection="3d")
        self.ax.clear()

        # draw the box
        box_x = [0, self.box.length, self.box.length, 0, 0]
        box_y = [0, 0, self.box.width, self.box.width, 0]
        box_z = [0, 0, 0, 0, 0]
        box_x2 = [0, self.box.length, self.box.length, 0, 0]
        box_y2 = [0, 0, self.box.width, self.box.width, 0]
        box_z2 = [self.box.height, self.box.height, self.box.height, self.box.height, self.box.height]
        self.ax.plot(box_x, box_y, box_z, color="b")
        self.ax.plot(box_x2, box_y2, box_z2, color="b")
        for i in range(4):
            self.ax.plot([box_x[i], box_x2[i]], [box_y[i], box_y2[i]], [box_z[i], box_z2[i]], color="b")

        legend_icons = []
        legend_labels = []

        # draw packed item
        for item in self.box.items_in_box:
            legend_icons.append(
                plt.Rectangle((0.5, 0.5), width=0.2, height=0.2, edgecolor=item.colour, facecolor=item.colour)
            )
            legend_labels.append(item.name)
            item_x_origin, item_y_origin, item_z_origin = item.coor
            item_length, item_width, item_height = item.get_rotated_dimension()
            self.ax.text(
                item_x_origin + 0.5 * item_length,
                item_y_origin + 0.5 * item_width,
                item_z_origin + 0.5 * item_height,
                item.name,
                color="black",
                fontsize=12,
            )
            item_x = [
                item_x_origin,
                item_x_origin + item_length,
                item_x_origin + item_length,
                item_x_origin,
                item_x_origin,
            ]
            item_y = [
                item_y_origin,
                item_y_origin,
                item_y_origin + item_width,
                item_y_origin + item_width,
                item_y_origin,
            ]
            item_z = [item_z_origin, item_z_origin, item_z_origin, item_z_origin, item_z_origin]

            item_x2 = [x for x in item_x]
            item_y2 = [y for y in item_y]
            item_z2 = [
                item_z_origin + item_height,
                item_z_origin + item_height,
                item_z_origin + item_height,
                item_z_origin + item_height,
                item_z_origin + item_height,
            ]
            for i in range(4):
                self.ax.plot_surface(
                    np.array([item_x[i : i + 2], item_x2[i : i + 2]]),
                    np.array([item_y[i : i + 2], item_y2[i : i + 2]]),
                    np.array([item_z[i : i + 2], item_z2[i : i + 2]]),
                    color=item.colour,
                    alpha=0.4,
                )
            self.ax.plot_surface(
                np.array([item_x, item_x2, item_x2[::-1], item_x[::-1], item_x]),
                np.array([item_y, item_y, item_y, item_y, item_y]),
                np.array([item_z, item_z2, item_z2, item_z, item_z]),
                color=item.colour,
                alpha=0.4,
            )

        # draw the axes, title, text etc
        self.ax.set_xlabel("X")
        self.ax.set_ylabel("Y")
        self.ax.set_zlabel("Z")
        self.ax.set_title(self.box.name)
        packing_detail_text = f"Successfully packed {len(self.box.items_in_box)} / {len(self.item_to_pack)} items"
        if self.packing_detail_text_component == None:
            self.packing_detail_text_component = self.fig.text(
                0.5,
                0.01,
                packing_detail_text,
                ha="center",
                va="bottom",
                fontsize=12,
            )
        else:
            self.packing_detail_text_component.set_text(packing_detail_text)
        plt.legend(handles=legend_icons, labels=legend_labels, loc="upper right")
        plt.draw()
        plt.pause(0.001)

    # another visualizer that shows how to pack one step at a time, for robot integration
    def visualization_by_step(self):
        if self.fig_by_step is None or self.ax_by_step is None:
            self.fig_by_step = plt.figure()
            self.ax_by_step = self.fig_by_step.add_subplot(111, projection="3d")
        self.ax_by_step.clear()

        # draw the box
        box_x = [0, self.box.length, self.box.length, 0, 0]
        box_y = [0, 0, self.box.width, self.box.width, 0]
        box_z = [0, 0, 0, 0, 0]
        box_x2 = [0, self.box.length, self.box.length, 0, 0]
        box_y2 = [0, 0, self.box.width, self.box.width, 0]
        box_z2 = [self.box.height, self.box.height, self.box.height, self.box.height, self.box.height]
        self.ax_by_step.plot(box_x, box_y, box_z, color="b")
        self.ax_by_step.plot(box_x2, box_y2, box_z2, color="b")
        for i in range(4):
            self.ax_by_step.plot([box_x[i], box_x2[i]], [box_y[i], box_y2[i]], [box_z[i], box_z2[i]], color="b")

        legend_icons = []
        legend_labels = []

        # draw packed item
        for index, item in enumerate(self.box.items_in_box):
            if index <= self.counter_by_step:
                legend_icons.append(
                    plt.Rectangle((0.5, 0.5), width=0.2, height=0.2, edgecolor=item.colour, facecolor=item.colour)
                )
                legend_labels.append(item.name)
                item_x_origin, item_y_origin, item_z_origin = item.coor
                item_length, item_width, item_height = item.get_rotated_dimension()
                self.ax_by_step.text(
                    item_x_origin + 0.5 * item_length,
                    item_y_origin + 0.5 * item_width,
                    item_z_origin + 0.5 * item_height,
                    item.name,
                    color="black",
                    fontsize=12,
                )
                item_x = [
                    item_x_origin,
                    item_x_origin + item_length,
                    item_x_origin + item_length,
                    item_x_origin,
                    item_x_origin,
                ]
                item_y = [
                    item_y_origin,
                    item_y_origin,
                    item_y_origin + item_width,
                    item_y_origin + item_width,
                    item_y_origin,
                ]
                item_z = [item_z_origin, item_z_origin, item_z_origin, item_z_origin, item_z_origin]

                item_x2 = [x for x in item_x]
                item_y2 = [y for y in item_y]
                item_z2 = [
                    item_z_origin + item_height,
                    item_z_origin + item_height,
                    item_z_origin + item_height,
                    item_z_origin + item_height,
                    item_z_origin + item_height,
                ]
                for i in range(4):
                    self.ax_by_step.plot_surface(
                        np.array([item_x[i : i + 2], item_x2[i : i + 2]]),
                        np.array([item_y[i : i + 2], item_y2[i : i + 2]]),
                        np.array([item_z[i : i + 2], item_z2[i : i + 2]]),
                        color=item.colour,
                        alpha=0.4,
                    )
                self.ax_by_step.plot_surface(
                    np.array([item_x, item_x2, item_x2[::-1], item_x[::-1], item_x]),
                    np.array([item_y, item_y, item_y, item_y, item_y]),
                    np.array([item_z, item_z2, item_z2, item_z, item_z]),
                    color=item.colour,
                    alpha=0.4,
                )

        # draw the axes, title, text etc
        self.ax_by_step.set_xlabel("X")
        self.ax_by_step.set_ylabel("Y")
        self.ax_by_step.set_zlabel("Z")
        self.ax_by_step.set_title(self.box.name)
        plt.legend(handles=legend_icons, labels=legend_labels, loc="upper right")

        plt.draw()
        plt.pause(0.001)


# initialise the Packer class, sort of like a packing machine
packing_machine = Packer()


# pytkinter calling function when 'Pack' button is called
def update_pack():
    box_data = eval(box_config.get().strip())
    packing_machine.reset_box()
    packing_machine.add_box(Box(*box_data))
    global previous_item_string

    # custom user input
    if use_custom_input_var.get():
        items_text = items_textbox.get("1.0", tk.END).strip()

        # reset not needed if previous item and curr is same, else reset the packing machine and start from zero
        if previous_item_string != items_text:
            previous_item_string = items_text
            packing_machine.reset_item()
            items_list = [line.strip() for line in items_text.split("\n") if line.strip()]
            for x in items_list:
                data = eval(x)
                item = Item(*data)
                packing_machine.add_item(item)

    # auto generate item
    else:
        # no need regenerate if user have the option checked
        if len(packing_machine.item_to_pack) and keep_curr_item_var.get():
            pass
        else:
            packing_machine.reset_item()
            number_of_random_item = int(random_textbox.get().strip())
            for i in range(number_of_random_item):
                packing_machine.add_item(
                    Item(
                        random.randint(1, 20),
                        random.randint(1, 20),
                        random.randint(1, 20),
                        f"item_{i+1}",
                    )
                )
    packing_machine.pack(
        consider_stability=consider_stability_var.get(),
        consider_gravity=consider_gravity_var.get(),
        enable_3d_rotation=enable_3d_rotation_var.get(),
    )


# pytkinter calling function when 'Show Next Packing Sequence' button is called
def show_packing_seq():
    packing_machine.counter_by_step += 1
    packing_machine.visualization_by_step()


# pytkinter calling function when 'Reset Packing Sequence' button is called
def reset_packing_seq():
    packing_machine.counter_by_step = 0
    packing_machine.visualization_by_step()


# pytkinter calling function when 'Reset' button is called
def reset_ui():
    global previous_item_string
    plt.close("all")
    previous_item_string = ""
    packing_machine.reset()


def toggle_input_textbox_visibility():
    if use_custom_input_var.get():
        items_label.pack()
        items_textbox.pack()
        random_textbox_label.pack_forget()
        random_textbox.pack_forget()
        keep_curr_checkbox.pack_forget()
    else:
        items_label.pack_forget()
        items_textbox.pack_forget()
        random_textbox_label.pack()
        random_textbox.pack()
        keep_curr_checkbox.pack()


# Pytkinter stuffs, display what to show on UI during which state
root = tk.Tk()
root.title("Packing Machine")
root.geometry("600x500")
consider_stability_var = tk.BooleanVar()
consider_gravity_var = tk.BooleanVar()
enable_3d_rotation_var = tk.BooleanVar()
use_custom_input_var = tk.BooleanVar(value=True)
keep_curr_item_var = tk.BooleanVar(value=True)

box_config_label = tk.Label(root, text="Box config (length, width, height), eg: (20, 20, 20)").pack()
box_config = tk.Entry(root)
box_config.insert(0, "(20, 20, 20, '20cm Box')")
box_config.pack()

tk.Checkbutton(root, text="Consider Stability", variable=consider_stability_var).pack()
tk.Checkbutton(root, text="Consider Gravity", variable=consider_gravity_var).pack()
tk.Checkbutton(root, text="Enable 3D rotation", variable=enable_3d_rotation_var).pack()
tk.Checkbutton(
    root, text="Use Custom Input", variable=use_custom_input_var, command=toggle_input_textbox_visibility
).pack()
keep_curr_checkbox = tk.Checkbutton(root, text="Keep current generated items", variable=keep_curr_item_var)

# Label and text entry field for items list
previous_item_string = ""
items_label = tk.Label(root, text="Items (one item per line):")
items_label.pack()
items_textbox = tk.Text(root, height=10, width=50)
items_textbox.pack()

random_textbox_label = tk.Label(root, text="Number of Items to ramdomly generate:")
random_textbox = tk.Entry(root)

tk.Button(root, text="Pack", command=update_pack).pack(side=tk.BOTTOM)
tk.Button(root, text="Show Next Packing Sequence", command=show_packing_seq).pack(side=tk.BOTTOM)
tk.Button(root, text="Reset Packing Sequence", command=reset_packing_seq).pack(side=tk.BOTTOM)
tk.Button(root, text="Reset", command=reset_ui).pack(side=tk.BOTTOM)

root.mainloop()
