#include <bits/stdc++.h>
#define f first
#define s second

using namespace std;

void le_entrada(ifstream &fin, int &num_of_cities, vector<pair<int,int>> &coordinates, vector<vector<int>> &distance_cost, vector<int> &cities)
{
    fin >> num_of_cities;
    distance_cost.assign(num_of_cities, vector<int>(num_of_cities, 0));

    for(int i = 0; i<num_of_cities; i++) 
    {
        int x, y;
        fin >> x >> y;
        coordinates.push_back(pair<int,int>(x,y));
        cities.push_back(i);
    }

    for(int i = 0; i<num_of_cities; i++)
    {
        for(int j = 0; j<num_of_cities; j++)
            fin >> distance_cost[i][j];
    }
    
}

void nearest_neighbor(int n, vector<vector<int>> &distance_cost, vector<int> &path, int &total_cost)
{
    srand(time(0));
    vector<bool> visited(n, false);
    total_cost = 0;
    int current_city = rand()%n;

    while(true)
    {
        path.push_back(current_city);
        visited[current_city] = true;
        int min_dist = INT_MAX; //pode falhar para distâncias muito grandes
        int next_city = -1;
        bool inicio = false;

        for(int i = 0; i<n; i++)
        {
            if(visited[i]) continue;
            if(distance_cost[current_city][i] < min_dist)
            {
                min_dist = distance_cost[current_city][i];
                next_city = i;
                inicio = false;
            }
            if(distance_cost[i][path[0]] < min_dist)
            {
                min_dist = distance_cost[i][path[0]];
                next_city = i;
                inicio = true;
            }
        }

        //cout << inicio << endl;
        //for(auto p : path) cout << p << " "; cout << endl;
        if(inicio) reverse(path.begin(), path.end());

        //for(auto p : path) cout << p << " "; cout << endl;
        //cout << path[path.size()-1] << " " << next_city << " " << distance_cost[path[path.size()-1]][next_city] << " " << min_dist << endl;

        if(next_city == -1) 
        {
            total_cost += distance_cost[path[n-1]][path[0]];
            path.push_back(path[0]);   
            break;
        }
        else current_city = next_city;
        total_cost += min_dist;
    }

    for(auto p : path) cout << p << " ";
    cout << endl;
    cout << "Cost: " << total_cost << endl;

}

void first_improvement_2opt(vector<int> &path, vector<vector<int>> &distance_cost, int n, int &cost)
{
    bool improved;
    do
    {
        improved = false;
        for(int j = 0; j<n-1; j++)
        {
            for(int k = j+3; k<n; k++) //para trocar 2 arestas, as cidades devem estar a pelo menos 3 ligações distantes
                {
                    
                    int current_cost = cost;
                    current_cost -= distance_cost[path[k]][path[k-1]];
                    current_cost -= distance_cost[path[j]][path[j+1]];
                    current_cost += distance_cost[path[j]][path[k-1]] + distance_cost[path[j+1]][path[k]];
                    
                    //cout << index_1 << " " << index_2 << endl;
                    //for(auto p : new_path) cout << p << " ";
                    //cout << endl;
                    //cout << current_cost << endl;
                    if(current_cost < cost)
                    {
                    vector<int> new_path;
                    int index_1 = j, index_2 = k;

                    for(int i = 0; i<=index_1; i++)
                        new_path.push_back(path[i]);

                    for(int i = index_2-1; i>index_1; i--)
                        new_path.push_back(path[i]);

                    for(int i = index_2; i<n; i++)
                        new_path.push_back(path[i]);

                    new_path.push_back(new_path[0]);

                    //cout << j << " " << k << endl;
                    //cout << "Now: \n";
                    //for(auto p : new_path) cout << p << " ";
                    //cout << "\nNew cost: " << current_cost << endl;
                    cost = current_cost;
                    path = new_path;
                    improved = true;
                }
                if(improved) break;
            }
            if(improved) break;
        }
    } while(improved);
}

void best_improvement_2opt(vector<int> &path, vector<vector<int>> &distance_cost, int n, int &cost)
{
    bool improved;
    do
    {
        improved = false;
        for(int j = 0; j<n-1; j++)
        {
            for(int k = j+3; k<n; k++)
                {
                    
                    int current_cost = cost;
                    current_cost -= distance_cost[path[k]][path[k-1]];
                    current_cost -= distance_cost[path[j]][path[j+1]];
                    current_cost += distance_cost[path[j]][path[k-1]] + distance_cost[path[j+1]][path[k]];
                    
                    //cout << index_1 << " " << index_2 << endl;
                    //for(auto p : new_path) cout << p << " ";
                    //cout << endl;
                    //cout << current_cost << endl;
                    if(current_cost < cost)
                    {
                    vector<int> new_path;
                    int index_1 = j, index_2 = k;

                    for(int i = 0; i<=index_1; i++)
                        new_path.push_back(path[i]);

                    for(int i = index_2-1; i>index_1; i--)
                        new_path.push_back(path[i]);

                    for(int i = index_2; i<n; i++)
                        new_path.push_back(path[i]);

                    new_path.push_back(new_path[0]);

                    //cout << j << " " << k << endl;
                    //cout << "Now: \n";
                    //for(auto p : new_path) cout << p << " ";
                    //cout << "\nNew cost: " << current_cost << endl;
                    cost = current_cost;
                    improved = true;
                    path = new_path;
                }
            }
        }
    } while(improved);
}

int main(int argc, char ** argv) // 1: first improvement, 2: best improvement
{

    ifstream fin("input.txt");
    int num_of_cities, total_cost;
    vector<vector<int>> distance_cost;
    vector<pair<int, int>> coordinates;
    vector<int> cities;

    vector<int>path;

    le_entrada(fin, num_of_cities, coordinates, distance_cost, cities);
    nearest_neighbor(num_of_cities, distance_cost, path, total_cost);
    if(argc > 1 && strcmp(argv[1], "1") == 0)
    {
        first_improvement_2opt(path, distance_cost, num_of_cities, total_cost);
    }
    else if(argc > 1 && strcmp(argv[1], "2") == 0)
    {
        best_improvement_2opt(path, distance_cost, num_of_cities, total_cost);
    }

    for(auto p : path) cout << p << " "; cout << endl; 
    cout << "Custo: " << total_cost << endl;

    return 0;
}