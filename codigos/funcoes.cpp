#include <bits/stdc++.h>

//all distances will be calculated as integers, so there will be some divergence on
//the calculus of some points

using namespace std;

void search(vector<int> cities, int cost, int &best_cost, vector<int> &best_path, int num_of_cities, vector<vector<int>> &distance_cost, vector<bool> &disponivel, vector<int> &points);

double calculateGeoDistance(double lat1,double long1,double lat2,double long2) {
    double dist;
    dist = sin(lat1) * sin(lat2) + cos(lat1) * cos(lat2) * cos(long1 - long2);
    dist = acos(dist);
    dist = (6371 * M_PI * dist) / 180;
    return dist;
}

double calculateEUC_2DDistance(double x1, double y1, double x2, double y2)
{
    double dist;
    double dx = abs(x2-x1);
    double dy = abs(y2-y1);
    dist = sqrt(dx * dx + dy * dy);
    return dist;
}

void EUC2D(vector<pair<double, double>> &coords, vector<vector<double>> &distance_cost)
{
    for(int i = 0; i<coords.size(); i++)
        for(int j = i+1; j<coords.size(); j++)
        {
            double dist = calculateEUC_2DDistance(coords[i].first, coords[i].second, coords[j].first, coords[j].second);
            distance_cost[i][j] = dist;
            distance_cost[j][i] = dist;
        }
}

void GEOD(vector<pair<double, double>> &coords, vector<vector<double>> &distance_cost)
{
    for(int i = 0; i<coords.size(); i++)
        for(int j = i+1; j<coords.size(); j++)
        {
            double dist = calculateGeoDistance(coords[i].first, coords[i].second, coords[j].first, coords[j].second);
            distance_cost[i][j] = dist;
            distance_cost[j][i] = dist;
        }
}

void readUpperRowMatrix(ifstream &fin, vector<vector<double>> &distance_cost, int n)
{
    for(int i = 0; i<n-1; i++)
        for(int j = i+1; j<n;j++)
        {
            fin >> distance_cost[i][j];
            distance_cost[j][i] = distance_cost[i][j];
        }
}

void readFullMatrix(ifstream &fin, vector<vector<double>> &distance_cost, int n)
{
    for(int i = 0; i<n; i++)
        for(int j = 0; j<n; j++)
            fin >> distance_cost[i][j];
}

void readLowerDiagRow(ifstream &fin, vector<vector<double>> &distance_cost, int n)
{
    for(int i = 0; i<n; i++)
        for(int j = 0; j<=i; j++)
        {
            fin >> distance_cost[i][j];
            distance_cost[j][i] = distance_cost[i][j];
        }
}

void readUpperDiagRow(ifstream &fin, vector<vector<double>> &distance_cost, int n)
{
    for(int i = 0; i<n; i++)
        for(int j = 0; j<n-1-i; j++)
        {
            fin >> distance_cost[i][j];
            distance_cost[j][i] = distance_cost[i][j];
        }
}

void readFromFile(string fileName, int &n, vector<pair<double, double>> &coordinates, vector<vector<double>> &distance_cost, string &problemName)
{
    ifstream fin("../tsplib/" + fileName);
    string line;
    if(!fin.is_open())
    {
        cout << "file is not open.\n";
        return;
    }

    string distance_type;
    string matrix_format;

    while(getline(fin, line))
    {
        auto pos = line.find(':') + 2;
        auto tam = line.size();
        if(line.substr(0, 4) == "NAME")                     problemName = line.substr(pos, tam-pos);
        else if(line.substr(0, 16) == "EDGE_WEIGHT_TYPE")   distance_type = line.substr(pos, tam-pos);
        else if(line.substr(0, 18) == "EDGE_WEIGHT_FORMAT") matrix_format = line.substr(pos, tam-pos);

        else if(line.substr(0, 9) == "DIMENSION")
        {
            n = stoi(line.substr(pos, tam-pos));
            coordinates.resize(n);
            distance_cost.resize(n, vector<double>(n));
        }

        else if(line.substr(0, 18) == "NODE_COORD_SECTION" || line.substr(0, 18) == "DISPLAY_DATA_SECTION")
        {
            int aux;
            for(int i = 0; i<n; i++) fin >> aux >> coordinates[i].first >> coordinates[i].second;
        }

        else if(line.substr(0, 19) == "EDGE_WEIGHT_SECTION")
        {
            int rowStart, colStart;
            if(matrix_format == "UPPER_ROW ")            readUpperRowMatrix(fin, distance_cost, n);
            else if(matrix_format == "FULL_MATRIX ")     readFullMatrix(fin, distance_cost, n);
            else if(matrix_format == "UPPER_DIAG_ROW ")  readUpperDiagRow(fin, distance_cost, n);
            else if(matrix_format == "LOWER_DIAG_ROW ")  readLowerDiagRow(fin, distance_cost, n);
        }
    }
    
    if(distance_type == "EUC_2D") EUC2D(coordinates, distance_cost);
    if(distance_type == "GEO") GEOD(coordinates, distance_cost);

    //for(auto p : coordinates) cout << p.first << " " << p.second << endl;

    //cout << problemName << endl;
    //cout << distance_type << endl;
    //cout << n << endl;

    //cout << distance_type << endl << matrix_format << endl;
    
    //for(auto row : distance_cost)
    //{
    //    for(auto col : row) cout << col << " ";
    //    cout << endl;
    //}
    
}

void nearest_neighbor(int n, vector<vector<double>> &distance_cost, vector<int> &path, long double &total_cost)
{
    srand(time(0));
    vector<bool> visited(n, false);
    total_cost = 0;
    int current_city = rand()%n;

    while(true)
    {
        path.push_back(current_city);
        visited[current_city] = true;
        double min_dist = numeric_limits<double>::max(); //pode falhar para distâncias muito grandes
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

    //for(auto p : path) cout << p << " ";
    //cout << endl;
    cout << "Cost: " << total_cost << endl;

}

void first_improvement_2opt(vector<int> &path, vector<vector<double>> &distance_cost, int n, long double &cost)
{
    bool improved;
    do
    {
        improved = false;
        for(int j = 0; j<n-1; j++)
        {
            for(int k = j+3; k<n; k++) //para trocar 2 arestas, as cidades devem estar a pelo menos 3 ligações distantes
                {
                    
                    double current_cost = cost;
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

void best_improvement_2opt(vector<int> &path, vector<vector<double>> &distance_cost, int n, long double &cost)
{
    bool improved;
    do
    {
        improved = false;
        for(int j = 0; j<n-1; j++)
        {
            for(int k = j+3; k<n; k++)
                {
                    
                    double current_cost = cost;
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


void recursion(vector<int> cities, double cost, long double &best_cost, vector<int> &best_path, int num_of_cities, vector<vector<double>> &distance_cost, vector<bool> &disponivel, vector<int> &points)
{
    if(cities.size() == num_of_cities)
    {
        cost += distance_cost[cities[num_of_cities-1]][cities[0]];
        cities.push_back(cities[0]);
        //for(auto c : cities)
        //        cout << c << " ";
        //    cout << endl << cost;
        //cout << endl;
        if(cost < best_cost)
        {
            best_cost = cost;
            best_path.clear();
            for(auto c : cities) best_path.push_back(c);
        }
        return;
    }

    if(cost > best_cost) return; // caso já esteja mais caro que o melhor, nem tenta mais

    for(int i = 0; i<num_of_cities; i++)
    {       
        if(disponivel[i])
        {
            cities.push_back(points[i]);
            disponivel[i] = false;
            if(cities.size() == 1)
                recursion(cities, cost, best_cost, best_path, num_of_cities, distance_cost, disponivel, points);
            else
                recursion(cities, cost + distance_cost[cities[cities.size()-2]][cities[cities.size()-1]], best_cost, best_path, num_of_cities, distance_cost, disponivel, points);
            disponivel[i] = true;
            cities.pop_back();
        }
    }

}