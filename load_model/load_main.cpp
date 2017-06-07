
#include <GL/gl.h>
#include <GL/glut.h>
#include <GL/glu.h>
#include <stdlib.h>
#include <math.h>
#include <iostream>

#include <string>
#include <cstring>

#include <vector>
#include <stdio.h>

#include <glm/glm.hpp>
#include <glm/vec3.hpp>
#include <glm/vec2.hpp>

using namespace std;

std::vector< glm::vec3 > vertices;
std::vector< glm::vec2 > uvs;
std::vector< glm::vec3 > normals;

/**
brief testing
*/
bool loadOBJ(
    const char * path, 
    std::vector<glm::vec3> & out_vertices,
    std::vector<glm::vec2> & out_uvs,
    std::vector<glm::vec3> & out_normals
){
    printf("Loading OBJ file %s...\n", path);

    std::vector< unsigned int > vertexIndices, uvIndices, normalIndices;
    //std::vector< unsigned int > vertexIndices;
	std::vector< glm::vec3 > temp_vertices;
	std::vector< glm::vec2 > temp_uvs;
	std::vector< glm::vec3 > temp_normals;

    FILE * file = fopen(path, "r");
    if( file == NULL ){
        printf("Impossible to open the file ! Are you in the right path ? See Tutorial 1 for details\n");
        return false;
    }

    while( 1 ){

        char lineHeader[128];
        // read the first word of the line
        int res = fscanf(file, "%s", lineHeader);
        if (res == EOF)
            break; // EOF = End Of File. Quit the loop.

        // else : parse lineHeader
        
        if ( strcmp( lineHeader, "v" ) == 0 ){
            glm::vec3 vertex;
            
            fscanf(file, "%f %f %f\n", &vertex.x, &vertex.y, &vertex.z );
            // cout<<"vertex x"<< vertex.x<<endl;
            // cout<<"vertex y"<<vertex.y<<endl;
            // cout<<"vertex z"<<vertex.z<<endl;
            temp_vertices.push_back(vertex);
        }

        else if ( strcmp( lineHeader, "vt" ) == 0 ){
	    glm::vec2 uv;
	    fscanf(file, "%f %f\n", &uv.x, &uv.y );
	    // cout<<"uv"<<uv.x<<endl;
	    temp_uvs.push_back(uv);
		} 
        else if ( strcmp( lineHeader, "vn" ) == 0 ){
            glm::vec3 normal;
            fscanf(file, "%f %f %f\n", &normal.x, &normal.y, &normal.z );
            // cout<<"normal x"<< normal.x<<endl;
            // cout<<"normal y"<< normal.y<<endl;
            // cout<<"normal z"<< normal.z<<endl;
            temp_normals.push_back(normal);
        }
        else if ( strcmp( lineHeader, "f" ) == 0 ){
    std::string vertex1, vertex2, vertex3;
    unsigned int vertexIndex[3], uvIndex[3], normalIndex[3];
    int matches = fscanf(file, "%d/%d/%d %d/%d/%d %d/%d/%d\n", &vertexIndex[0], &uvIndex[0], &normalIndex[0], &vertexIndex[1], &uvIndex[1], &normalIndex[1], &vertexIndex[2], &uvIndex[2], &normalIndex[2] );
    if (matches != 9){
        printf("File can't be read by our simple parser : ( Try exporting with other options\n");
        return false;
    }
    vertexIndices.push_back(vertexIndex[0]);
    vertexIndices.push_back(vertexIndex[1]);
    vertexIndices.push_back(vertexIndex[2]);
    uvIndices    .push_back(uvIndex[0]);
    uvIndices    .push_back(uvIndex[1]);
    uvIndices    .push_back(uvIndex[2]);
    normalIndices.push_back(normalIndex[0]);
    normalIndices.push_back(normalIndex[1]);
    normalIndices.push_back(normalIndex[2]);
            
        }
    }


    // For each vertex of each triangle
    for( unsigned int i=0; i<vertexIndices.size(); i++ ){

        // Get the indices of its attributes
        unsigned int vertexIndex = vertexIndices[i];
        
        // Get the attributes thanks to the index
        glm::vec3 vertex = temp_vertices[ vertexIndex-1 ];
        
        // Put the attributes in buffers
        out_vertices.push_back(vertex);
    
    }
    cout<<"size"<< out_vertices.size()<<endl;
    printf("loaded file %s successfully.\n", path);
    return true;
}


void debug() {
    
 
    bool res = loadOBJ("ellipse.obj", vertices, uvs, normals);
    //bool res = loadOBJ("ellipse.obj", vertices);
    
    puts("VERTEX______________________________________________________");
    
    // for (unsigned int i = 0; i < vertices.size(); i++) {
    //     cout << vertices[i].x << vertices[i].y << vertices[i].z << endl;   
    // }
    cout<<"finish debug"<<endl; 
}

int main(int argc, char** argv) {

/****test for glm::mat3****/
	//  glm::mat3 I;
	// 	for(int i(0); i<3; i++){
	// 	for(int j(0);j<3;j++){
	// 		I[i][j] = 0.0;
	// 	}
	// }
	// I[0][0] = 1;
	// I[1][1] = 1;
	// I[2][2] = 1;

	// cout<<I[0][0]<<endl;
	// cout<<I[0][1]<<endl;

	// 	 glm::mat3 I2;
	// 	for(int i(0); i<3; i++){
	// 	for(int j(0);j<3;j++){
	// 		I2[i][j] = 0.0;
	// 	}
	// }
	// I2[0][0] = 1;
	// I2[1][1] = 1;
	// I2[2][2] = 1;

	// glm::mat3 I3 = I * I2;

	// cout<<I3[0][0]<<endl;
	// cout<<I3[0][1]<<endl;

/***start debug loading***/
    debug();
    //bool obj = loadOBJ("ellipse.obj",vertices, uvs, normals);


    return 0;
}