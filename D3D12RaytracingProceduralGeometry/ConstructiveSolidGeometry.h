#pragma once


struct TreeNode {
	int value;
	TreeNode* left;
	TreeNode* right;
};


class ConstructiveSolidGeometry
{
	TreeNode* root;
	//we represent CSG by a tree - this is then flattened to an array and uploaded to the GPU.
public:
	ConstructiveSolidGeometry();
private:
};

