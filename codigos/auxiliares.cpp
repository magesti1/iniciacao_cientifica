#include <bits/stdc++.h>

//all distances will be calculated as integers, so there will be some divergence on
//the calculus of some points

using namespace std;

int calculateGeoDistance(int lat1, int long1, int lat2, int long2) {
    double dist;
    dist = sin(lat1) * sin(lat2) + cos(lat1) * cos(lat2) * cos(long1 - long2);
    dist = acos(dist);
    dist = (6371 * M_PI * dist) / 180;
    return dist;
}

int calculateEUC_2DDistance(int x1, int y1, int x2, int y2)
{
    int dist;
    dist = abs(x2-x1) * abs(x2-x1) + abs(y2-y1) + abs(y2-y1);
    return dist;
}

void EUC2D(vector<pair<int,int>> &coords, vector<vector<int>> &distance_cost)
{
    for(int i = 0; i<coords.size(); i++)
        for(int j = i+1; j<coords.size(); j++)
        {
            int dist = calculateEUC_2DDistance(coords[i].first, coords[i].second, coords[j].first, coords[j].second);
            distance_cost[i][j] = dist;
            distance_cost[j][i] = dist;
        }
}