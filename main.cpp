#include <iostream>
#include <list>
#include <algorithm>

using namespace std;
template<typename E>//E = edge struct
class AbstractClass{
public:
    virtual int numVertices() = 0;
    virtual list<int> getVertices() = 0 ;
    virtual int numEdges() = 0;
    virtual list<E> getEdges() = 0 ;
    virtual E getEdge(int u,int v) = 0;
    virtual pair<int,int> endVertices(E e) = 0 ;
    virtual int opposite(int v,E e) = 0;
    virtual int outDegree(int v) = 0 ;
    virtual int inDegree(int v) = 0;
    virtual list<E> outgoingEdges(int v) = 0;
    virtual list<E> incomingEdges(int v) = 0;
    virtual void insertVertex(int x) = 0 ;
    virtual void insertEdge(int u,int v,int x) = 0 ;
    virtual void removeVertex(int v) = 0 ;
    virtual void removeEdge(E e) = 0;
};

struct Edge {
    int src, dest;
};
struct GraphEdge {
    Edge edge;
    list<GraphEdge>::iterator position;
};
struct Vertex {
    int value;
    list<Vertex>::iterator position;
};

class Graph : public AbstractClass<Edge>{
private:

    list<Vertex> vertices;
    list<GraphEdge> edges;
public:
    int numVertices() override{
        return vertices.size();
    }

    list<int> getVertices() override{
        list<int> verticesList;
        for (const auto& vertex : vertices) {
            verticesList.push_back(vertex.value);
        }
        return verticesList;
    }

    int numEdges() override{
        return edges.size();
    }

    list<Edge> getEdges() override{
        list<Edge> edgesList;
        for (const auto& graphEdge : edges) {
            edgesList.push_back(graphEdge.edge);
        }
        return edgesList;
    }

    Edge getEdge (int u, int v)  override {
        auto it = find_if(edges.begin(), edges.end(), [&](const GraphEdge& graphEdge) {
            return graphEdge.edge.src == u && graphEdge.edge.dest == v;
        });
        if (it != edges.end()) {
            return it->edge;
        }
        return { -1, -1 };  // Return invalid edge if not found
    }

    pair<int,int> endVertices(Edge e) override {
        for (const auto& edge : edges) {
            if (edge.edge.src == e.src || edge.edge.dest == e.dest) {
                return {e.src,e.dest} ;
            }
        }
        return { -1, -1 }; // Return invalid edge if vertex is not found
    }

    int opposite(int v, Edge e) override{
        if (e.src == v) {
            return e.dest;
        }
        else if (e.dest == v) {
            return e.src;
        }
        return -1;  // Return -1 if v is not an end vertex of e
    }

    int outDegree(int v) override{
        int count = 0;
        for (const auto& graphEdge : edges) {
            if (graphEdge.edge.src == v) {
                count++;
            }
        }
        return count;
    }

    int inDegree(int v) override{
        int count = 0;
        for (const auto& graphEdge : edges) {
            if (graphEdge.edge.dest == v) {
                count++;
            }
        }
        return count;
    }

    list<Edge> outgoingEdges(int v) override{
        list<Edge> outgoing;
        for (const auto& graphEdge : edges) {
            if (graphEdge.edge.src == v) {
                outgoing.push_back(graphEdge.edge);
            }
        }
        return outgoing;
    }

    list<Edge> incomingEdges(int v) override{
        list<Edge> incoming;
        for (const auto& graphEdge : edges) {
            if (graphEdge.edge.dest == v) {
                incoming.push_back(graphEdge.edge);
            }
        }
        return incoming;
    }

    void insertVertex(int x) override{
        vertices.push_back({ x });
        vertices.back().position = --vertices.end();
    }

    void insertEdge(int u, int v,int x) override{
        edges.push_back({ { u, v } });
        edges.back().position = --edges.end();
    }

    void removeVertex(int v) override{
        // Remove edges incident on vertex v
        auto it = edges.begin();
        while (it != edges.end()) {
            if (it->edge.src == v || it->edge.dest == v) {
                it = edges.erase(it);
            }
            else {
                ++it;
            }
        }

        // Remove vertex v
        auto vertexIt = find_if(vertices.begin(), vertices.end(), [&](const Vertex& vertex) {
            return vertex.value == v;
        });
        if (vertexIt != vertices.end()) {
            vertices.erase(vertexIt);
        }

        // Update positions of vertices after v
        for (auto it = vertices.begin(); it != vertices.end(); ++it) {
            it->position = it;
        }
    }

    void removeEdge(Edge e) override{
        auto it = find_if(edges.begin(), edges.end(), [&](const GraphEdge& graphEdge) {
            return graphEdge.edge.src == e.src && graphEdge.edge.dest == e.dest;
        });
        if (it != edges.end()) {
            edges.erase(it);
        }
    }
};

int main() {
    
}