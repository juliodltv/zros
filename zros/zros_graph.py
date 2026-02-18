import networkx as nx
import matplotlib.pyplot as plt
from zros import Node

class ZrosGraph(Node):
    def __init__(self):
        super().__init__("zros_graph_visualizer")
        self.create_subscriber("_zros/graph", self.graph_callback)
        self.nodes = {}  # {node_name: {publishers: [], subscribers: []}}
        
        # Setup plotting
        plt.ion()
        self.fig, self.ax = plt.subplots(figsize=(12, 8))
        self.fig.canvas.manager.set_window_title("zros_graph")
        self.create_timer(2.0, self.update_plot)

    def graph_callback(self, payload):
        """
        Payload: {"name": str, "publishers": [str], "subscribers": [str]}
        """
        name = payload.get("name")
        if name:
            self.nodes[name] = {
                "publishers": payload.get("publishers", []),
                "subscribers": payload.get("subscribers", [])
            }

    def update_plot(self):
        self.ax.clear()
        G = nx.DiGraph()

        # Build graph
        # Nodes are ellipses, Topics are boxes
        topics = set()
        
        for node_name, info in self.nodes.items():
            # Hide self (zros_graph_visualizer)
            if node_name == self.name:
                continue

            G.add_node(node_name, type='node', color='lightblue')
            
            for topic in info['publishers']:
                if topic.startswith("_zros"): continue # Skip internal topics
                topics.add(topic)
                G.add_edge(node_name, topic)
            
            for topic in info['subscribers']:
                if topic.startswith("_zros"): continue
                topics.add(topic)
                G.add_edge(topic, node_name)

        for topic in topics:
            G.add_node(topic, type='topic', color='lightgreen')

        if len(G.nodes) == 0:
            self.ax.text(0.5, 0.5, "Waiting for nodes...", ha='center')
            plt.pause(0.1)
            return

        # Layout
        pos = nx.spring_layout(G, seed=42)
        
        # Draw Nodes
        node_nodes = [n for n, attr in G.nodes(data=True) if attr.get('type') == 'node']
        nx.draw_networkx_nodes(G, pos, nodelist=node_nodes, node_color='lightblue', node_shape='o', node_size=2000, ax=self.ax)
        
        # Draw Topics
        topic_nodes = [n for n, attr in G.nodes(data=True) if attr.get('type') == 'topic']
        nx.draw_networkx_nodes(G, pos, nodelist=topic_nodes, node_color='lightgreen', node_shape='s', node_size=1500, ax=self.ax)
        
        # Draw Edges and Labels
        nx.draw_networkx_edges(G, pos, ax=self.ax, arrows=True, arrowstyle='-|>', arrowsize=20, node_size=2000)
        nx.draw_networkx_labels(G, pos, ax=self.ax)
        
        # self.ax.set_title("ZROS Network Graph")
        self.ax.axis('off') # Hide axes for cleaner look
        plt.pause(0.1)

def main():
    node = ZrosGraph()
    try:
        node.spin()
    except KeyboardInterrupt:
        pass
    finally:
        plt.close()

if __name__ == "__main__":
    main()
