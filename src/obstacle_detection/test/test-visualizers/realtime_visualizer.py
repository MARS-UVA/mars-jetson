import socket
import threading
import queue
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
import numpy as np
import time
import traceback
import re

class PathVisualizationServer:
    def __init__(self, host='127.0.0.1', port=12345):
        self.plotted_vertices = False
        self.host = host
        self.port = port
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.bind((self.host, self.port))
        self.running = True
        
        # Message queue for thread-safe communication
        self.message_queue = queue.Queue()
        
        # For visualization
        plt.ion()  # Turn on interactive mode
        self.fig, self.ax = plt.subplots(figsize=(10, 8))
        self.ax.set_title('Path Planning Visualization')
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.grid(True)
        plt.tight_layout()
        
        # Store explored edges and final path
        self.explored_edges = []
        self.final_path = []
        
        # For determining when to update the plot
        self.last_update_time = time.time()
        self.needs_update = False
        
        # Debug flags
        self.debug = True
    
    def debug_print(self, message):
        """Print debug messages if debug is enabled"""
        if self.debug:
            print(f"[DEBUG] {message}")
    
    def start(self):
        # Start server in a separate thread
        server_thread = threading.Thread(target=self.run_server)
        server_thread.daemon = True
        server_thread.start()
        
        # Run visualization loop in the main thread
        try:
            self.run_visualization_loop()
        except KeyboardInterrupt:
            print("Keyboard interrupt received, shutting down...")
        finally:
            self.shutdown()
    
    def run_server(self):
        """Handle server operations in a separate thread"""
        self.server_socket.listen(1)
        print(f"Server started on {self.host}:{self.port}")
        
        while self.running:
            try:
                # Set a timeout so we can check if the server is still running
                self.server_socket.settimeout(1.0)
                try:
                    client_socket, addr = self.server_socket.accept()
                    print(f"Connected by {addr}")
                    client_thread = threading.Thread(target=self.handle_client, args=(client_socket, addr))
                    client_thread.daemon = True
                    client_thread.start()
                except socket.timeout:
                    continue
            except Exception as e:
                print(f"Server error: {e}")
                traceback.print_exc()
                if not self.running:
                    break
        
        print("Server thread exiting")
    
    def handle_client(self, client_socket, addr):
        """Handle client messages in a separate thread"""
        buffer = ""
        try:
            client_socket.settimeout(0.5)  # Set timeout for receiving
            
            # Print a message when client connects
            print(f"Client {addr} connected. Waiting for data...")
            
            # Add a test point to verify visualization works
            self.explored_edges.append(((0, 0), (1, 1)))
            self.needs_update = True
            
            while self.running:
                try:
                    data = client_socket.recv(1024)
                    if not data:
                        self.debug_print(f"Client {addr} sent empty data, disconnecting")
                        break
                    
                    # Print raw data for debugging
                    data_str = data.decode('utf-8', errors='replace')
                    self.debug_print(f"Received raw data from {addr}: {data_str}")
                    
                    buffer += data_str
                    
                    # Look for command patterns in the buffer
                    # First, try to find EXPLORE command patterns
                    explore_pattern = r'EXPLORE,(-?\d+\.?\d*),(-?\d+\.?\d*),(-?\d+\.?\d*),(-?\d+\.?\d*)'
                    explore_matches = re.findall(explore_pattern, buffer)
                    
                    for match in explore_matches:
                        current_x, current_y, parent_x, parent_y = map(float, match)
                        self.explored_edges.append(((parent_x, parent_y), (current_x, current_y)))
                        self.needs_update = True
                        print(f"Processed EXPLORE: {current_x}, {current_y}, {parent_x}, {parent_y}")
                    
                    # Remove processed EXPLORE commands from buffer
                    buffer = re.sub(explore_pattern, '', buffer)
                    
                    # Try to find PATH command patterns
                    # path_pattern = r'PATH,(-?\d+\.?\d*),(-?\d+\.?\d*)'
                    # path_matches = re.findall(path_pattern, buffer)
                    
                    # for match in path_matches:
                    #     x, y = map(float, match)
                    #     self.final_path.append((x, y))
                    #     self.needs_update = True
                    #     print(f"Processed PATH: {x}, {y}")
                    
                    # # Remove processed PATH commands from buffer
                    # buffer = re.sub(path_pattern, '', buffer)
                    
                    # Twist
                    path_pattern = r'TWIST,(-?\d+\.?\d*),(-?\d+\.?\d*),(-?\d)'
                    path_matches = re.findall(path_pattern, buffer)

                    import heapq
                    # Create a priority queue (min heap) for the TWIST commands
                    twist_queue = []
                    for match in path_matches:
                        linVel, angVel, order = map(float, match)
                        heapq.heappush(twist_queue, (order, (linVel, angVel)))
                        print(f"Processed TWIST: {linVel}, {angVel}")

                    # Process the priority queue
                    # while twist_queue:
                    #     order, (x, y) = heapq.heappop(twist_queue)
                    #     self.final_path.append((x, y))
                    #     self.needs_update = True
                    #     print(f"Processed TWIST: {x}, {y}, order: {order}")
                    
                    # for match in path_matches:
                    #     x, y, order = map(float, match)
                    #     self.final_path.append((x, y))
                    #     self.needs_update = True
                    #     print(f"Processed PATH: {x}, {y}")
                    
                    # Check for TERMINATE command
                    if 'TERMINATE' in buffer:
                        print("Received termination signal")
                        self.running = False
                        self.needs_update = True
                        buffer = buffer.replace('TERMINATE', '')

                        ## Read a PLY file and put points in  final_path list
                    # try:
                    #     from plyfile import PlyData
                    #     import numpy as np
                        
                    #     path_ply_path = "./src/obstacle_detection/test/assets/path_python1.ply"  # REPLACE WITH YOUR PATH PLY FILE
                    #     print(f"Loading path PLY file: {path_ply_path}")
                        
                    #     path_plydata = PlyData.read(path_ply_path)
                    #     path_vertices = path_plydata['vertex'].data
                        
                    #     # Clear existing path and add new points from PLY
                    #     self.final_path = []
                    #     for i in range(len(path_vertices)):
                    #         x = path_vertices['x'][i]
                    #         y = path_vertices['y'][i]
                    #         self.final_path.append((x, y))
                        
                    #     print(f"Added {len(self.final_path)} points from PLY file to final path")
                    # except Exception as e:
                    #     print(f"Error loading path PLY file: {e}")
                    #     traceback.print_exc()
                    
                    # Try to find floating-point numbers that might be part of coordinates
                    # This is a fallback for messages that don't match the expected format
                    if len(buffer.strip()) > 0:
                        coord_pattern = r'(-?\d+\.?\d*)'
                        coords = re.findall(coord_pattern, buffer)
                        
                        if len(coords) >= 4 and len(coords) % 2 == 0:
                            # Process as pairs of coordinates
                            for i in range(0, len(coords) - 2, 2):
                                try:
                                    x1, y1 = float(coords[i]), float(coords[i+1])
                                    x2, y2 = float(coords[i+2]), float(coords[i+3])
                                    self.explored_edges.append(((x2, y2), (x1, y1)))
                                    self.needs_update = True
                                    print(f"Processed raw coordinates: ({x1}, {y1}) from ({x2}, {y2})")
                                except (ValueError, IndexError):
                                    pass
                            
                            # Clear the buffer after processing
                            buffer = ""
                        elif len(coords) >= 2:
                            # Process as a single point
                            try:
                                for i in range(0, len(coords), 2):
                                    if i + 1 < len(coords):
                                        x, y = float(coords[i]), float(coords[i+1])
                                        self.final_path.append((x, y))
                                        self.needs_update = True
                                        print(f"Processed raw point: ({x}, {y})")
                            except ValueError:
                                pass
                            
                            # Clear the buffer after processing
                            buffer = ""
                    
                    # Keep the buffer size manageable
                    if len(buffer) > 1024:
                        buffer = buffer[-1024:]
                    
                except socket.timeout:
                    continue
                except Exception as e:
                    self.debug_print(f"Error receiving data from {addr}: {e}")
                    traceback.print_exc()
                    break
        except Exception as e:
            print(f"Client handler error: {e}")
            traceback.print_exc()
        finally:
            client_socket.close()
            print(f"Client {addr} disconnected")
    
    def process_queue(self):
        """Process all messages in the queue"""
        processed = 0
        max_per_cycle = 100  # Process at most 100 messages per cycle
        
        while not self.message_queue.empty() and processed < max_per_cycle:
            try:
                message = self.message_queue.get_nowait()
                self.process_message(message)
                self.message_queue.task_done()
                processed += 1
            except queue.Empty:
                break
        
        return processed
    
    def process_message(self, message):
        """Process a message from the queue (this is now a backup method)"""
        try:
            # Try to parse as a structured message
            parts = message.split(',')
            cmd = parts[0]
            
            if cmd == "EXPLORE" and len(parts) >= 5:
                # Format: EXPLORE,currentX,currentY,parentX,parentY
                print(f"Received explore message: {message}")
                try:
                    current_x, current_y = float(parts[1]), float(parts[2])
                    parent_x, parent_y = float(parts[3]), float(parts[4])
                    self.explored_edges.append(((parent_x, parent_y), (current_x, current_y)))
                    self.needs_update = True
                except ValueError:
                    print(f"Invalid coordinates in message: {message}")
            
            elif cmd == "PATH" and len(parts) >= 3:
                # Format: PATH,x,y
                print(f"Received path message: {message}")
                try:
                    x, y = float(parts[1]), float(parts[2])
                    self.final_path.append((x, y))
                    self.needs_update = True
                except ValueError:
                    print(f"Invalid coordinates in message: {message}")
            
            elif cmd == "TERMINATE":
                print("Received termination signal")
                self.running = False
                self.needs_update = True
            
            # Unknown command but still has structure
            elif len(parts) >= 3:
                try:
                    # Try to parse as raw coordinate data
                    x1, y1 = float(parts[0]), float(parts[1])
                    
                    if len(parts) >= 4:  # Has parent coordinates
                        x2, y2 = float(parts[2]), float(parts[3])
                        self.explored_edges.append(((x2, y2), (x1, y1)))
                    else:
                        # Just a point for the path
                        self.final_path.append((x1, y1))
                    
                    self.needs_update = True
                    print(f"Parsed raw coordinate data: {message}")
                except ValueError:
                    print(f"Could not parse as coordinates: {message}")
            
            else:
                print(f"Unknown message format: {message}")
        
        except Exception as e:
            print(f"Error processing message: {e}")
            traceback.print_exc()
    
    def run_visualization_loop(self):
        """Main visualization loop - must run in the main thread for Matplotlib"""
        update_interval = 0.1  # Update at most 10 times per second
        
        while self.running:
            # Process messages from the queue
            messages_processed = self.process_queue()
            
            # Update visualization if needed
            current_time = time.time()
            if (self.needs_update or messages_processed > 0 or 
                current_time - self.last_update_time > 1.0):  # Force update every second
                self.update_visualization()
                self.last_update_time = current_time
                self.needs_update = False
            
            # Process GUI events
            plt.pause(0.01)
        
        # Final update and keep the window open
        self.update_visualization()
        print("Algorithm complete. Close the plot window to exit.")
        plt.ioff()
        plt.show(block=True)
    
    def update_visualization(self):
        """Update the plot with current data"""
        # Clear the plot
        self.ax.clear()
        self.ax.grid(True)

        ## Plot ply file here, hardcoded ply file
        try:
            if not hasattr(self, 'ply_loaded') or not self.ply_loaded:
                # Load the PLY file on first visualization update
                from plyfile import PlyData
                import numpy as np
                
                ply_path = "./src/obstacle_detection/test/assets/vertices_python.ply"  # REPLACE WITH YOUR ACTUAL PLY FILE PATH
                print(f"Loading PLY file: {ply_path}")
                
                plydata = PlyData.read(ply_path)
                vertices = plydata['vertex'].data
                
                self.ply_points_x = vertices['x']
                self.ply_points_y = vertices['y']
                self.ply_points_z = vertices['z'] if 'z' in vertices.dtype.names else np.zeros_like(self.ply_points_x)
                
                self.ply_loaded = True
                print(f"PLY file loaded successfully with {len(self.ply_points_x)} points")
            
            # Plot the PLY points as background, color-coded by Z height using viridis colormap
            scatter_bg = self.ax.scatter(
                self.ply_points_x, 
                self.ply_points_y, 
                c=self.ply_points_z,  # This makes the color depend on Z value
                cmap='viridis',       # This sets the colormap to viridis
                s=1,                  # Small point size
                alpha=0.2,            # Low opacity
                label='PLY Background'
            )
            
        except Exception as e:
            print(f"Error loading or plotting PLY file: {e}")
            import traceback
            traceback.print_exc()
        
        # Plot explored edges
        for (px, py), (cx, cy) in self.explored_edges:
            if(px < 1.5 and px > -0.6 and cx < 1.5 and cx > -0.6 and py > 0.0 and cy > 0.0 and py <= 1.6 and cy <= 1.6):
                self.ax.plot([px, cx], [py, cy], 'c-', alpha=0.3, linewidth=1)
        
        # Plot final path if available
        # if len(self.final_path) > 1:
        #     path_x = [p[0] for p in self.final_path if -0.6 <= p[0] <= 1.5 and p[1] > 0.0 and p[1] <= 1.6]
        #     path_y = [p[1] for p in self.final_path if -0.6 <= p[0] <= 1.5 and p[1] > 0.0 and p[1] <= 1.6]
        #     self.ax.plot(path_x, path_y, 'r-', linewidth=2)
            
            # Mark start and goal
            # if self.final_path:
            #     self.ax.plot(self.final_path[0][0], self.final_path[0][1], 'go', markersize=10)
            #     self.ax.plot(self.final_path[-1][0], self.final_path[-1][1], 'ro', markersize=10)
        
        # Add legend
        legend_elements = [
            Line2D([0], [0], color='c', alpha=0.3, linewidth=1, label='Explored'),
            Line2D([0], [0], color='r', linewidth=2, label='Final Path'),
            Line2D([0], [0], marker='o', color='g', label='Start', markersize=10, linestyle='None'),
            Line2D([0], [0], marker='o', color='r', label='Goal', markersize=10, linestyle='None')
        ]
        self.ax.legend(handles=legend_elements, loc='upper right')
        
        # Update plot title with statistics
        self.ax.set_title(f'Path Planning Visualization\n'
                        f'Explored: {len(self.explored_edges)} edges, '
                        f'Path length: {len(self.final_path)} nodes')
        
        # Redraw the canvas
        self.fig.canvas.draw()
    
    def shutdown(self):
        """Clean shutdown of all resources"""
        self.running = False
        self.server_socket.close()
        print("Server shut down")

if __name__ == "__main__":
    server = PathVisualizationServer()
    server.start()