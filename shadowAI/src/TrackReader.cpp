//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
//

#include "TrackReader/TrackReader.h"
#include <float.h>

using namespace std;

TrackReader::TrackReader(string str)
{
	read(str);
	createIterators();
}

TrackReader::~TrackReader(void)
{
	delete [] iterator_array;
}

void TrackReader::read(string str) {
	ifstream readFile(str, ifstream::in);
	char* fileName = const_cast<char*>(str.c_str());
	char* fullLine = new char[445050];
	if (!readFile){
		cout << "Error opening file" << endl;
	}
	if (readFile.is_open()) {
		//cout<<"opened"<<endl;
		bool found_total = false;

		readFile.getline(fullLine, 445050);
		stringstream tokenizer(fullLine);
		num_persons;
		while (true){
			if(isdigit(tokenizer.peek())) {
				tokenizer >> num_persons;
				break;
			}
			else{
				tokenizer.ignore(500, ' ');
			}
		}
		cout<<"Number of Persons: "<<num_persons<<endl;
		
		float minTime = FLT_MAX;	//the trajectories are not strictly ordered in the files, so the timeChange needs to be applied in a separate pass
		
		for (int i = 0; i < num_persons; i++){
			vector<Point> all_points;
			readFile.getline(fullLine, 445050);
			stringstream tokenizer2(fullLine);
			
			while(tokenizer2.peek() == EOF){
				readFile.getline(fullLine, 445050);
				tokenizer2.clear();
				tokenizer2.str(fullLine);
			}
			/*
			int points;
			float start_time;
			float end_time;
			
			while (true) {
				if(isdigit(tokenizer2.peek())) {
					tokenizer2 >> points >> start_time >> end_time;
					break;
				}
				else{
					tokenizer2.ignore(500, '[');
				}
			}
			*/
			readFile.getline(fullLine, 445050);
			tokenizer2.clear();
			tokenizer2.str(fullLine);
			
			float x, z, time;
			while (tokenizer2.peek() != EOF) {
				if(isdigit(tokenizer2.peek())) {
					tokenizer2 >> x >> z >> time;

					if(time < minTime){
						minTime = time;
					}
					Point p = Point(x*.0247f, time,z*.0247f); 
					all_points.push_back(p);
				}
				else {
					tokenizer2.ignore(500, '[');
				}
			}
			
			all_trajectories.push_back(all_points);
		}

		for(vector<vector<Point>>::iterator iter = all_trajectories.begin(); iter != all_trajectories.end(); ++iter) {
			for(vector<Point>::iterator piter = iter->begin(); piter != iter->end(); ++piter) {
				piter->y -= minTime;
			}
		}
	}

	delete [] fullLine;
}

void TrackReader::createIterators(){
	iterator_array = new vector<Point>::iterator[num_persons];
	for (int i = 0; i<all_trajectories.size(); i++){
		vector<Point>::iterator it = all_trajectories[i].begin();
		iterator_array[i] = it;
	}
}

//number of persons
int TrackReader::num_trajectory(void) const {
	return num_persons;
}

//entry time for a person
float TrackReader::entry_time(int id) const {
	return all_trajectories[id][0].y;
}

//exit time for a person
float TrackReader::exit_time(int id) const {
	int size = all_trajectories[id].size();
	return all_trajectories[id][size - 1].y;
}

//entry location for a person
Point TrackReader::entry_location(int id) const {
	float start_spot_x = all_trajectories[id][0].x;
	float start_spot_z = all_trajectories[id][0].z;
	return Point(start_spot_x, 0.0, start_spot_z);
}

//exit location for a person
Point TrackReader::exit_location(int id) const {
	int size = all_trajectories[id].size();
	float start_spot_x = all_trajectories[id][size - 1].x;
	float start_spot_z = all_trajectories[id][size - 1].z;
	return Point(start_spot_x, 0.0, start_spot_z);
}
//current position for a person
Point TrackReader::current_location(int id, float time) {
	vector<Point>::iterator temp;
	for (temp = iterator_array[id]; temp != all_trajectories[id].end(); ++temp){
		if(temp->y >= time) {
			break;
		}
	}
	if(temp == all_trajectories[id].end()) {
		temp--;
	}
	iterator_array[id] = temp;
	return Point(temp->x, 0.0f, temp->z);
}

Vector TrackReader::velocity(int id) {
	float time = iterator_array[id]->y;

	//Jennie: What i replaced here was a well thought-out piece of code you came up with, but it depends on their data being reliable,
	//unfortunately their tracking data does at time "skip" samples, so 0.5 seconds was almost always, but not universally, 5 elements away
	//as a result, you got some out-of-bounds errors on the vector accesses.
	vector<Point>::iterator lower_bound; 
	vector<Point>::iterator upper_bound;

	for(lower_bound = iterator_array[id]; lower_bound != all_trajectories[id].begin(); --lower_bound) {
		//this loop will quit when it hits this conditional break statement, or when it hits the beginning of the array
		if(lower_bound->y < time - 5.0f) {
			break;
		}
	}

	for(upper_bound = iterator_array[id]; upper_bound != all_trajectories[id].end(); ++upper_bound) {
		//this loop will quit when it hits the conditional break statement OR after passing the end, so an additional check will be needed
		if(upper_bound->y > time + 5.0f) {
			break;
		}
	}
	if(upper_bound == all_trajectories[id].end()) {
		upper_bound--;
	}

	float change_in_x = upper_bound->x - lower_bound->x;
	float change_in_z = upper_bound->z - lower_bound->z;
	float change_in_time = upper_bound->y - lower_bound->y;

	//instead of overly complex logic to guarantee a 1 second window, if the above is off by a little the division will take normalize it
	return Vector(change_in_x / change_in_time, change_in_time / change_in_time, change_in_z / change_in_time);
}
	