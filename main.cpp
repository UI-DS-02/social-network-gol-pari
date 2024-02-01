#include <iostream>
#include <list>
#include <algorithm>
#include <vector>
#include <unordered_map>
#include <map>
#include <set>
#include <fstream>
#include <string>
#include <sstream>
#include <functional>
#include <nlohmann/json.hpp>

using namespace std;
using json = nlohmann::json;

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
class Edge_list : public AbstractClass<Edge>{
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

struct Edge2 {
    int src;
    int dest;
};
class Adjacency_map :AbstractClass<Edge2>{
private:
    unordered_map<int, list<Edge2>> adjacencyMap;

public:
    void insertVertex(int v) override{
        adjacencyMap[v] = {};
    }

    void insertEdge(int u, int v,int x) override{
        adjacencyMap[u].push_back({ u, v });
    }

    void removeVertex(int v) override{
        // Remove outbound edges
        for (auto& adjList : adjacencyMap) {
            adjList.second.remove_if([v](Edge2 edge) {
                return edge.dest == v;
            });
        }

        // Remove vertex
        adjacencyMap.erase(v);
    }

    void removeEdge(Edge2 e) override{
        if (adjacencyMap.find(e.src) != adjacencyMap.end()) {
            adjacencyMap[e.src].remove_if([&e](Edge2 edge) {
                return edge.dest == e.dest;
            });
        }
    }

    int numVertices() {
        return adjacencyMap.size();
    }

    list<int> getVertices() {
        list<int> vertices;
        for (const auto& p : adjacencyMap) {
            vertices.push_back(p.first);
        }
        return vertices;
    }

    int numEdges() {
        int count = 0;
        for (const auto& p : adjacencyMap) {
            count += p.second.size();
        }
        return count;
    }

    list<Edge2> getEdges() {
        list<Edge2> edges;
        for (const auto& p : adjacencyMap) {
            edges.insert(edges.end(), p.second.begin(), p.second.end());
        }
        return edges;
    }

    Edge2 getEdge(int u, int v) {
        if (adjacencyMap.find(u) != adjacencyMap.end()) {
            auto it = find_if(adjacencyMap[u].begin(), adjacencyMap[u].end(), [v](Edge2 edge) {
                return edge.dest == v;
            });
            if (it != adjacencyMap[u].end()) {
                return *it;
            }
        }
        return { -1, -1 };
    }

    pair<int, int> endVertices(Edge e) {
        return make_pair(e.src, e.dest);
    }

    int opposite(int v, Edge e) {
        return e.src == v ? e.dest : e.src;
    }

    int outDegree(int v) {
        if (adjacencyMap.find(v) != adjacencyMap.end()) {
            return adjacencyMap[v].size();
        }
        return 0;
    }

    int inDegree(int v) {
        int count = 0;
        for (const auto& p : adjacencyMap) {
            for (const auto& edge : p.second) {
                if (edge.dest == v) {
                    count++;
                }
            }
        }
        return count;
    }

    list<Edge2> outgoingEdges(int v) {
        if (adjacencyMap.find(v) != adjacencyMap.end()) {
            return adjacencyMap[v];
        }
        return {};
    }

    list<Edge2> incomingEdges(int v) {
        list<Edge2> incoming;
        for (const auto& p : adjacencyMap) {
            for (const auto& edge : p.second) {
                if (edge.dest == v) {
                    incoming.push_back({ p.first, edge.dest });
                }
            }
        }
        return incoming;
    }
};

class Adjacency_list : public AbstractClass<pair<int,int>>
{
public:
    int num_vertices;
    vector<vector<int>> adj_list;
    int num_edges;
    Adjacency_list() //constructor
    {
        num_vertices = 0;
        num_edges = 0;
    }

    int numVertices() override//return the number of vertices of the graph
    {
        return num_vertices;
    }

    list<int> getVertices() override//return an iteration of all the vertices of the graph
    {
        list<int> vertices_list;
        for (int i = 0; i < num_vertices; i++)
        {
            vertices_list.push_back(adj_list[i][0]);
        }
        return vertices_list;
    }

    int numEdges() override // return the number of edges of the graph
    {
        return num_edges;
    }

    list<pair<int,int>> getEdges() override //return an iteration of all the edges of the graph
    {
        list<pair<int,int>> edges;
        for (int i = 0; i < num_vertices; i++)
        {
            for (int j = 1; j < adj_list[i].size(); j++)
            {
                edges.push_back(make_pair(adj_list[i][0],adj_list[i][j]));
            }
        }
        return edges;
    }

    pair<int,int> getEdge(int u, int v) override //return the edge from vertex u to v
    {
        for (int i = 0; i < num_vertices; i++)
        {
            if (adj_list[i][0] == u)
            {
                for(int j=1 ; j<adj_list[i].size() ; j++)
                {
                    if(adj_list[i][j] == v)
                        return make_pair(u , v);
                }
                break;
            }
        }
        return make_pair(-1 , -1);
    }

    pair<int, int> endVertices(pair<int, int> e) override//return the two endpoint of vertices of edge e
    {
        return make_pair(e.first,e.second);
    }

    int opposite(int v, pair<int, int> e) override//return the other vertex of the edge
    {
        pair<int,int> vertex = endVertices(e);
        if(v == vertex.first)
            return vertex.second;
        else if(v == vertex.second)
            return vertex.first;
        return -1;
    }

    int outDegree(int v) override// return the number of outgoing edges from vertex v
    {
        for (int i = 0; i < num_vertices; i++)
        {
            if (adj_list[i][0] == v)
            {
                return adj_list[v].size()-1;
            }
        }
        return -1;
    }

    int inDegree(int v) override// return the number of incoming edges to vertex v
    {
        return outDegree(v);
    }

    list<pair<int,int>> outgoingEdges(int v) override//return an iteration of all outgoing edges from vertex v
    {
        list<pair<int,int>> outgoing_edges_list;
        for (int i = 0; i < num_vertices ; i++)
        {
            if(adj_list[i][0] == v)
            {
                for(int j=1 ; j<adj_list[i].size() ; j++)
                {
                    outgoing_edges_list.push_back(make_pair(adj_list[i][0],adj_list[i][j]));
                }
                return outgoing_edges_list;
            }
        }
        return outgoing_edges_list;
    }

    list<pair<int,int>> incomingEdges(int v) override// return an iteration of all incoming edges to vertex v
    {
        return outgoingEdges(v);
    }

    void insertVertex(int x) override// create a new vertex with element x
    {
        vector<int> new_vertex = {x};
        adj_list.push_back(new_vertex);
        num_vertices++;
    }

    void insertEdge(int u, int v, int x) override// create a new edge from vertex u to v with element x
    {
        for (int i = 0; i < num_vertices; i++)
        {
            if(adj_list[i][0] == u)
            {
                adj_list[i].push_back(v);
                break;
            }
        }
        for (int i = 0; i < num_vertices; i++)
        {
            if(adj_list[i][0] == v)
            {
                adj_list[i].push_back(u);
                break;
            }
        }
        num_edges++;
    }

    void removeVertex(int v) override// remove vertex v and all its edges from graph
    {
        int temp;
        for(int i=0 ; i<num_vertices ; i++)
        {
            if(adj_list[i][0] == v)
            {
                temp=i;
                break;
            }
        }
        adj_list.erase(adj_list.begin() + temp);
        num_vertices--;
        for (int i = 0; i < num_vertices; i++)
        {
            for (int j = 0; j < adj_list[i].size(); j++)
            {
                if (adj_list[i][j] == v)
                {
                    adj_list[i].erase(adj_list[i].begin() + j);
                    num_edges--;
                }
            }
        }
    }

    void removeEdge(pair<int, int> e) override// removes edge e
    {
        for(int j=0 ; j<num_vertices ; j++)
        {
            if(e.first == adj_list[j][0])
            {
                for (int i = 1; i < adj_list[j].size(); i++)
                {
                    if(e.second == adj_list[j][i])
                        adj_list[j].erase(adj_list[j].begin() + i);
                }
            }
        }
        for(int j=0 ; j<num_vertices ; j++)
        {
            if(e.second == adj_list[j][0])
            {
                for (int i = 1; i < adj_list[j].size(); i++)
                {
                    if(e.first == adj_list[j][i])
                        adj_list[j].erase(adj_list[j].begin() + i);
                }
            }
        }
        num_edges--;
    }
};

class Adjacency_matrix : public AbstractClass<int>
{
public:
    int num_Vertices;
    int num_edges;
    vector<vector<int>> adjMatrix;
    Adjacency_matrix()
    {
        num_Vertices = 0;
        num_edges = 0;
        vector<int> newmat = {0};
        adjMatrix.push_back(newmat);
    }
    int numVertices() override//return the number of vertices of the graph
    {
        return num_Vertices;
    }
    list<int> getVertices() override
    {
        list<int> vertices;
        for (int i = 1 ; i < num_Vertices; i++)
        {
            vertices.push_back(adjMatrix[i][0]);
        }
        return vertices;
    }
    int numEdges() override // return the number of edges of the graph
    {
        return num_edges;
    }
    list<int> getEdges() override //return an iteration of all the edges of the graph
    {
        list<int> edges;
        for (int i = 1 ; i < num_Vertices; i++)
        {
            for (int j = 1; j < adjMatrix[i].size(); j++)
            {
                if (adjMatrix[i][j] != 0)
                {
                    edges.push_back(adjMatrix[i][j]);
                }
            }
        }
        return edges;
    }
    pair<int, int> endVertices(int e) override//return the two endpoint of vertices of edge e
    {
        for (int i = 1 ; i < num_Vertices; i++)
        {
            for (int j = 1 ; j < num_Vertices; j++)
            {
                if (adjMatrix[i][j] == e)
                {
                    return make_pair(adjMatrix[i][0],adjMatrix[0][j]);
                }
            }
        }
        return make_pair(-1, -1);
    }
    int opposite(int v, int e) override//return the other vertex of the edge
    {
        pair<int,int> vertex = endVertices(e);
        if(v == vertex.first)
            return vertex.second;
        else if(v == vertex.second)
            return vertex.first;
        return -1;
    }
    int outDegree(int v) override// return the number of outgoing edges from vertex v
    {
        for (int i = 1; i < num_Vertices; i++)
        {
            if (adjMatrix[i][0] == v)
            {
                int count = 0;
                for (int j = 1; j < num_Vertices; j++)
                {
                    if (adjMatrix[i][j] != 0)
                    {
                        count++;
                    }
                }
                return count;
            }
        }
        return -1;
    }
    int inDegree(int v) override// return the number of incoming edges to vertex v
    {
        return outDegree(v);
    }
    list<int> outgoingEdges(int v) override//return an iteration of all outgoing edges from vertex v
    {
        list<int> outgoing_edges_list;
        for (int i=1 ; i < num_Vertices ; i++)
        {
            if(adjMatrix[i][0] == v)
            {
                for (int j=1 ; j < num_Vertices; j++)
                {
                    if (adjMatrix[i][j] != 0)
                    {
                        outgoing_edges_list.push_back(adjMatrix[i][j]);
                    }
                }
                return outgoing_edges_list ;
            }
        }
        return outgoing_edges_list ;
    }
    list<int> incomingEdges(int v) override// return an iteration of all incoming edges to vertex v
    {
        return outgoingEdges(v);
    }
    void insertVertex(int x) override// create a new vertex with element x
    {
        vector<int> new_vertex = {x};
        new_vertex.resize(num_Vertices+1 , 0);
        adjMatrix.push_back(new_vertex);
        adjMatrix[0].push_back(x);
        num_Vertices++;
    }
    void insertEdge(int u, int v, int x) override// create a new edge from vertex u to v with element x
    {
        for (int i = 0; i < num_Vertices ; i++)
        {
            if(adjMatrix[i][0] == u)
            {
                u = i;
                break;
            }
        }
        for (int i = 0; i < num_Vertices ; i++)
        {
            if(adjMatrix[0][i] == v)
            {
                v = i;
                break;
            }
        }
        adjMatrix[u][v] = x;
        adjMatrix[v][u] = x;
        num_edges++;
    }
    void removeVertex(int v) override// remove vertex v and all its edges from graph
    {
        int temp;
        for(int i=0 ; i< num_Vertices ; i++)
        {
            if(adjMatrix[i][0] == v)
            {
                temp=i;
                break;
            }
        }
        for(int i=0 ; i< num_Vertices ; i++)
        {
            if (adjMatrix[i][temp] != 0)
            {
                adjMatrix[i][temp] = 0 ;
                num_edges -- ;
            }
        }
        adjMatrix.erase(adjMatrix.begin() + temp);
        adjMatrix[0].erase(adjMatrix[0].begin()+temp);
        num_Vertices--;
    }
    void removeEdge(int e) override// removes edge e
    {
        for(int j=1 ; j <num_Vertices ; j++)
        {
            for(int i=1 ; i<num_Vertices ; i++)
            {
                if(adjMatrix[j][i] == e)
                {
                    adjMatrix[j][i] = 0 ;
                }
            }
        }
        num_edges--;
    }
};

class user
{
public:
    int id ;
    string name ;
    string dateOfBirth;
    string universityLocation ;
    string field ;
    string workplace ;
    vector<string> specialties ;
    vector<int> connection;
    int weight ;
    user()
    {
        this->weight = 0;
    }
    bool operator > (const user& u) const
    {
        return (this->weight > u.weight);
    }
    void operator = (const user & other)
    {
        this->id = other.id ;
        this->name = other.name ;
        this->dateOfBirth = other.dateOfBirth ;
        this->universityLocation = other.universityLocation ;
        this->field = other.field ;
        this->workplace = other.workplace ;
        this->specialties = other.specialties ;
        this->connection = other.connection ;
        this->weight = other.weight ;
    }
};
map<int,user> users;

set<int> weight(set<int> id , int person)
{
    user per = users[person];
    for(auto ID : id)
    {
        user info = users[ID] ;
        for (auto x : per.specialties)
            for (auto j : info.specialties)
                if (x == j)
                    info.weight += 7; //Considering heavy weight for specialties
        if(info.field == per.field)
            info.weight += 3;
        if(info.workplace == per.workplace)
            info.weight += 2;
        if(info.universityLocation == per.universityLocation)
            info.weight += 2;
    }
    return id ;
}
vector<int> searchForRelevant(vector<int> id, Adjacency_list graph , int w){
    vector<int> children ;
    for(int i=0 ; i < id.size() ; i++){
        list<pair<int, int>> edges  = graph.outgoingEdges(id[i]) ;
        for (const auto& edge : edges) {
            int child = graph.opposite(id[i], edge);
            users[child].weight += w ;
            children.push_back(child);
        }
    }
    return children;
}
set<int> makeListOfRelevant(int id,Adjacency_list graph){
    set<int> suggestions ;
    vector<int> temp;
    temp.push_back(id);
    for(int i = 5 ; i >= 1 ; i--){
        temp = searchForRelevant(temp, graph, i);
        if(temp.size() > 0){
            for(auto i:temp){
                suggestions.insert(i);
            }
        }
        else
            break;
    }
    return suggestions;
}
void menu()
{
    cout << "1:Enter the desired person's ID to get 20 suggestions to connect" << endl;
    cout << "2:Add a new person to the social network" << endl;
    cout << "3:Exit" << endl ;
}

int main()
{
    ifstream json_file("D:\\UNI\\SEMASTER3\\Ramezani\\json\\users.json");
    nlohmann::json people;
    json_file >> people;
    Adjacency_list graph ;
    for (const auto& person : people)
    {
        user x ;
        x.id = stoi(person["id"].get<std::string>());
        x.name = person["name"].get<std::string>() ;
        x.dateOfBirth = person["dateOfBirth"].get<std::string>() ;
        x.universityLocation = person["universityLocation"].get<std::string>() ;
        x.field = person["field"].get<std::string>() ;
        x.workplace = person["workplace"].get<std::string>() ;
        for (const auto& specialty : person["specialties"])
        {
            x.specialties.push_back(specialty.get<std::string>());
        }
        for (const auto& connection : person["connectionId"])
        {
            x.connection.push_back(stoi(connection.get<std::string>()));
        }
        graph.insertVertex(x.id);
        for (const auto& connection : x.connection)
        {
            list<int> vertices = graph.getVertices();
            bool find = false ;
            for (auto x : vertices)
            {
                if(x == connection) //if the vertex already existed
                {
                    find = true ;
                    break;
                }
            }
            if (find == false) // if the vertex is not existed , insert it
                graph.insertVertex(connection);
            graph.insertEdge(x.id,connection,0);
        }
        users[x.id] = x;
    }
    menu();
    int order ;
    cin >> order ;
    while (order != 3)
    {
        if (order == 1)
        {
            cout << "Enter the id" << endl ;
            int id ;
            cin >> id ;
            set<int> bfs = makeListOfRelevant(id, graph);
            set<int> suggest = weight(bfs,id);
            vector<user> all_suggest ;
            for(auto sugg : suggest)
            {
                user per = users[sugg];
                all_suggest.push_back(per);
            }
            sort(all_suggest.begin(), all_suggest.end(), greater<user>());
            user x = users[id];
            int count = 0;
            for(auto sugg : all_suggest)
            {
                if(count != 20)
                {
                    bool find = false ;
                    for (auto j : x.connection)
                    {
                        if(sugg.id == j)
                        {
                            find = true;
                            break;
                        }
                    }
                    if(find == false)
                    {
                        cout << "name : " << sugg.name << " | id : " << sugg.id << endl ;
                        count += 1;
                    }
                }
            }
        }
        else if (order == 2)
        {
            user x ;
            cout << "ID : " << endl ;
            cin >> x.id ;
            while(users.count(x.id) == 1){
                cout<<"Sorry!This id already taken,Please try again:)\n";
                cin >> x.id ;
            }
            cout << "Name : " << endl ;
            cin  >> x.name ;
            cout << "DateOfBirth : " << endl ;
            cin  >> x.dateOfBirth ;
            cout << "UniversityLocation : " << endl ;
            cin  >> x.universityLocation ;
            cout << "Field : " << endl ;
            cin  >> x.field ;
            cout << "Workplace : " << endl ;
            cin  >> x.workplace ;
            cout << "What field do you specialize in?" << endl;
            string line;
            cin>>line;
            x.specialties.push_back(line);
            int order2;
            cout << "Enter 1 if you want add more specialties\n";
            cin>>order2;
            while(order2 == 1){
                cin >> line;
                cout << "Enter 1 if you want add more specialties\n";
                cin >> order2;
                x.specialties.push_back(line);
            }
            users.insert(make_pair(x.id,x));
            graph.insertVertex(x.id);
            cout << "Registration was successful\nThese are my suggestions for you" << endl;
            list<int> vertices = graph.getVertices();
            set<int> tempset(vertices.begin(),vertices.end());
            tempset = weight(tempset,x.id);
            vector<user> all_suggest ;
            for(auto sugg : tempset)
            {
                user per = users[sugg];
                all_suggest.push_back(per);
            }
            sort(all_suggest.begin(), all_suggest.end(), greater<user>());
            int count = 0;
            for(auto sugg : all_suggest)
            {
                if(count != 20)
                {
                    cout << "name : " << sugg.name << " | id : " << sugg.id << endl ;
                    count += 1;
                }
            }
            cout << "If you want to follow someone, please enter their ID, otherwise press zero" << endl;
            int id;
            cin >> id;
            while(id != 0){
                cin >> id;
                x.connection.push_back(id);
            }
            for (const auto& connection : x.connection){
                graph.insertEdge(x.id,connection,0);
            }
        }
        menu();
        cin >> order ;
    }
}