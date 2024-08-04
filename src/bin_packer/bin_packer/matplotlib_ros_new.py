import matplotlib.pyplot as plt
import matplotlib.animation as animation
import datetime as dt

# Initialize lists to store the data
x_data = []
y_data = []

# Set the initial y value
y_value = 10

# Function to update the plot
def update(frame):
    global y_value
    print("hellow from update")
    x_data.append(dt.datetime.now().strftime('%H:%M:%S'))
    y_data.append(y_value)
    y_value += 1  # Increment the y value by 1
    
    # Clear the current plot
    ax.clear()
    
    # Plot the new data
    ax.plot(x_data, y_data, label="y = time")
    
    # Format the plot
    ax.set_title("Random Line Chart")
    ax.set_xlabel("Time")
    ax.set_ylabel("Y-axis")
    ax.legend()
    ax.grid(True)
    
    # Rotate and align the x labels
    plt.xticks(rotation=45, ha='right')

# Create a figure and axis
fig, ax = plt.subplots()

# Create an animation
ani = animation.FuncAnimation(fig, update, interval=1000)  # Update every 1000 milliseconds (1 second)

# Show the plot
plt.show()
