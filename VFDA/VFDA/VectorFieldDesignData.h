#pragma once
#include "Visualize.h"
#include "Design.h"

#include "IsInClosedRegion.h"
#include <map>
#include <set>
#include <queue>
#include "Smooth.h"
#include "sample_curve.h"

extern std::vector<float> color_table;

template<typename T>
class SingleFrameDesignData {
	VectorField<T> *vField = 0;
	Eigen::Matrix<T, -1, -1> VF;	
public:	
	std::vector<float> vx, vy;

	//设计元素
	vector<std::vector<float>> sketching_xss, sketching_yss;	
	std::vector<VFDesign_Element<T>*> eles;
	vector<Eigen::Matrix<T, Dynamic, Dynamic>> sketchs;
	//中间数据 
	vector<std::vector<int>> triangles_on_sketchs;
	vector<std::vector<int>> nearest_point_indices_to_triangles_on_sketchs;

	vector< std::map<int, vector<std::pair<int, T>> > >
		nearest_k_points_indies_dists_to_triangleTrips_on_sketchs;

	set<int>  fixed_vertices;

public:
	SingleFrameDesignData(VectorField<T> *vField = 0,const int image_width=512,const int image_height=512) {
		set(vField, image_width, image_height);
		generate_noise_image();
	}
	SingleFrameDesignData(const SingleFrameDesignData &sfda) {
		vField = sfda.vField;
		for (int i = 0; i < sfda.eles.size(); i++) {
			if (sfda.eles[i]->type < 1 || sfda.eles[i]->type>5) {
				VFDesign_ResularElement<T> *ele = static_cast<VFDesign_ResularElement<T> *>(sfda.eles[i]);
				VFDesign_ResularElement<T> *ele2 = new VFDesign_ResularElement<T>(*ele);
				eles.push_back((VFDesign_Element<T>*)ele);
			}
			else {
				VFDesign_SingularElement<T> *ele = static_cast<VFDesign_SingularElement<T> *>(sfda.eles[i]);
				VFDesign_SingularElement<T> *ele2 = new VFDesign_SingularElement<T>(*ele);
				eles.push_back((VFDesign_Element<T>*)ele);
			}			
		}
		sketchs = sfda.sketchs;

		//----------中间数据---------- 
		triangles_on_sketchs = sfda.triangles_on_sketchs;
		nearest_point_indices_to_triangles_on_sketchs = sfda.nearest_point_indices_to_triangles_on_sketchs;
		nearest_k_points_indies_dists_to_triangleTrips_on_sketchs = sfda.nearest_k_points_indies_dists_to_triangleTrips_on_sketchs;
		fixed_vertices = sfda.fixed_vertices;
		VF = sfda.VF;

		vx = sfda.vx;  vy = sfda.vy;
		singularities = sfda.singularities;
		image_width = sfda.image_width;
		image_height = sfda.image_height;
		noise_img = sfda.noise_img;
		LIC_img = sfda.LIC_img;
		glTex_img = sfda.glTex_img;
	}
	void set(VectorField<T> *vField, const int image_width = 512, const int image_height = 512) {
		this->vField = vField; 
		this->image_width = image_width;  this->image_height = image_height;
		if (vField) {
			VF.resize(vField->VF.rows(), vField->VF.cols());
			VF.setZero(vField->VF.rows(), vField->VF.cols());
		}
	}

	void add_singularity_element(const Eigen::Matrix<T, 1, -1> &pos, const int type,
		const T k = 1, const T d = 0.0004) {
		VFDesign_SingularElement<T>* singular = new VFDesign_SingularElement<T>
			(pos, d, type, k);
		eles.push_back((VFDesign_Element<T>*)singular);
		compute_vectors_for_element(eles.size() - 1);
		update();// smooth();
	}
	void add_regularity_element(const Eigen::Matrix<T, 1, -1> &pos, const int type, 
		const Eigen::Matrix<T, 1, -1> &vec, const T d = 0.0004) {
		VFDesign_ResularElement<T> *resular = new VFDesign_ResularElement<T>(pos, d, type, vec);
		eles.push_back((VFDesign_Element<T>*)resular);
		compute_vectors_for_element(eles.size() - 1);
		update();// smooth();
	}
	void add_sketch(const std::vector<T> xs, const std::vector<T> &ys) {
		assert(xs.size() == ys.size());
		Eigen::Matrix<T, Dynamic, Dynamic> sketch(xs.size(), 2);
		for (int i = 0; i < xs.size(); i++) {
			sketch(i, 0) = xs[i]; sketch(i, 1) = ys[i];
		}
		sketchs.push_back(sketch);

		//extract triangle strip for this sketch
		//nearest_point_indices  are nearest points on the sketch to these triangles
		vector<int> triangles, nearest_point_indices;
		extract_sketch_triangle_strip(*vField, sketch, triangles, nearest_point_indices);
		triangles_on_sketchs.push_back(triangles);
		nearest_point_indices_to_triangles_on_sketchs.push_back(nearest_point_indices);

		compute_vectors_for_sketch_triangle_strip(sketchs.size() - 1);

		update();// smooth();
	}	

	void compute_vectors_for_sketch_triangle_strip(const int sketch_idx)
	{
		const int k = 5;
		const int s = sketch_idx;
		const vector<int> &triangles = triangles_on_sketchs[s];
		const vector<int> &nearest_point_indices = nearest_point_indices_to_triangles_on_sketchs[s];
		const Eigen::Matrix<T, Dynamic, Dynamic> &sketch = sketchs[s];
		std::map<int, std::set<int> > nearest_k_points_indies;
		std::map<int, vector<std::pair<int, T>> > nearest_k_points_indies_dists;

		std::ofstream oF("k_points_indies_.txt");
		if (!oF) return;

		////collect nearest_points to a vertex
		for (int i = 0; i<triangles.size(); i++) {
			//collect the k_points_indies for the triangle
			int idx = nearest_point_indices[i];  //the idx-point on sketch is in the triangle
			std::vector<int> k_points_indies;
			for (int j = -k; j <= k; j++) {
				int idx2 = (j + idx + sketch.rows()) % sketch.rows();
				k_points_indies.push_back(idx2);
			}
			for (const auto& idx : k_points_indies)
				oF << idx << " ";
			oF << "\n";

			int f = triangles[i];
			int nvertices = vField->F.cols();
			for (int j = 0; j<nvertices; j++) {
				int vid = vField->F(f, j);

				for (const auto& idx : k_points_indies) {
					nearest_k_points_indies[vid].insert(idx);
				}
			}
		}
		oF.close();
		
		//compute distances of nearest_points to a vertex
		const int dim = vField->V.cols();
		Eigen::Matrix<T, 1, Dynamic> P(1, dim);
		Eigen::Matrix<T, 1, Dynamic> V(1, dim);
		Eigen::Matrix<T, 1, Dynamic> VP(1, dim);
		const int num = 2 * k + 1;

		for (const auto& kv : nearest_k_points_indies) {
			int vid = kv.first;
			std::set<int> nearest_point_indices = kv.second;
			vector<std::pair<int, T>> nearest_point_indices_distances;
			V = vField->V.row(vid);
			for (const auto& idx : nearest_point_indices) {
				P = sketch.row(idx);
				VP = P - V;
				T distance = VP*VP.adjoint();
				nearest_point_indices_distances.push_back(std::pair<int, T>(idx, distance));
			}
			sort(nearest_point_indices_distances.begin(), nearest_point_indices_distances.end(),
				[](const std::pair<int, T> &a, const std::pair<int, T> &b) {return a.second < b.second; });

			if (nearest_point_indices_distances.size() > num)
				nearest_point_indices_distances.erase(nearest_point_indices_distances.begin() + num,
					nearest_point_indices_distances.end());
			nearest_k_points_indies_dists[vid] = nearest_point_indices_distances;
		}

		nearest_k_points_indies_dists_to_triangleTrips_on_sketchs.push_back(nearest_k_points_indies_dists);

		//---computer vectors for vertices in the triangletrip---
		//first, computer vectors for points on the sketch
		Eigen::Matrix<T, Dynamic, Dynamic> sketch_vectors(sketch.rows(), sketch.cols());
		Eigen::Matrix<T, 1, Dynamic> preP(1, dim);
		Eigen::Matrix<T, 1, Dynamic> nextP(1, dim);
		Eigen::Matrix<T, 1, Dynamic> preV, pV;
		preP = sketch.row(sketch.rows() - 1);
		P = sketch.row(0);
		preV = P - preP;
		for (int i = 0; i < sketch.rows(); i++) {
			nextP = sketch.row((i + 1) % sketch.rows());
			V = nextP - P;
			pV = (preV + V);
			pV.normalize();
			sketch_vectors.row(i) = pV;
			//for next step in the loop
			preP = P;  P = nextP;  preV = V;
		}
		//second,computer vectors for vertices in the triangletrip
		T d = 100.;
		for (const auto & kv : nearest_k_points_indies_dists) {
			int vid = kv.first;
			Eigen::Matrix<T, 1, Dynamic> V(1, vField->VF.cols());
			V.setZero(1, vField->VF.cols());
			Eigen::Matrix<T, 1, Dynamic> Q = vField->V.row(vid);
			const vector<std::pair<int, T>> &near_points = kv.second;

			for (const auto & pair : near_points) {
				int idx = pair.first;
				P = sketch.row(idx);
				pV = sketch_vectors.row(idx);
				Eigen::Matrix<T, 1, Dynamic> PQ = Q - P;
				T length2 = PQ*PQ.adjoint();
				V += exp(-d*length2)*pV;
			}
			V.normalize();
			vField->VF.row(vid) = V;

			fixed_vertices.insert(vid); //add the vertex to the fixed vertices
		}
	}
	void compute_vectors_for_sketch_triangle_strip() {
		fixed_vertices.clear();
		for (int s = 0; s < triangles_on_sketchs.size(); s++)
			compute_vectors_for_sketch_triangle_strip(s);
	}
	void compute_vectors_for_elements() {
		if (eles.size() == 0) return;
		
		for (int i = 0; i < eles.size(); i++) {
			compute_vectors_for_element(i);
		}
	}
	void compute_vectors_for_element(const int ele_id) {
		VFDesign_Element<T>* ele = eles[ele_id];
		int triangle_id = TriangleDetect(*vField, ele->p(0), ele->p(1));
		assert(triangle_id >= 0);

		int v0 = vField->F(triangle_id, 0), v1 = vField->F(triangle_id, 1), v2 = vField->F(triangle_id, 2);
		Eigen::Matrix<T, 1, Dynamic> P0 = vField->V.row(v0);
		Eigen::Matrix<T, 1, Dynamic> P1 = vField->V.row(v1);
		Eigen::Matrix<T, 1, Dynamic> P2 = vField->V.row(v2);

		vector<T> q = { ele->p(0), ele->p(1) };
		vector<T> alpha(3);
		vField->GetBarycentricFacters(triangle_id, q, alpha);
		float alpah_min = 1e-8, alpah_max = 1 - 1e-8;
		if (alpha[0]<alpah_min || alpha[0]>alpah_max
			|| alpha[1]<alpah_min || alpha[1]>alpah_max
			|| alpha[2]<alpah_min || alpha[2]>alpah_max
			) {
			float alp = 1. / 3;
			alpha[0] = alp; alpha[1] = alp; alpha[2] = alp;
			ele->p(0) = alpha[0] * P0(0) + alpha[1] * P1(0) + alpha[2] * P2(0);
			ele->p(1) = alpha[0] * P0(1) + alpha[1] * P1(1) + alpha[2] * P2(1);
		}
		Eigen::Matrix<T, 1, Dynamic> O(1, vField->VF.cols());
		O(0) = ele->p(0); O(1) = ele->p(1);
		Eigen::Matrix<T, 1, Dynamic> V0 = P0 - O;
		Eigen::Matrix<T, 1, Dynamic> V1 = P1 - O;
		Eigen::Matrix<T, 1, Dynamic> V2 = P2 - O;
		V0.normalize(); V1.normalize(); V2.normalize();


		if (ele->type < 1 || ele->type>5) {
			VFDesign_ResularElement<T> *ele_regular = (VFDesign_ResularElement<T> *)ele;
			if (ele->type == 6) {
				V0(0) = 1; V0(1) = 0;  V1(0) = 1; V1(1) = 0; V2(0) = 1; V2(1) = 0;
			}
			else if (ele->type == 7) {
				V0(0) = -1; V0(1) = 0;  V1(0) = -1; V1(1) = 0; V2(0) = -1; V2(1) = 0;
			}
			else if (ele->type == 8) {
				V0(0) = 0; V0(1) = 1;  V1(0) = 0; V1(1) = 1; V2(0) = 0; V2(1) = 1;
			}
			else if (ele->type == 9) {
				V0(0) = 0; V0(1) = -1;  V1(0) = 0; V1(1) = -1; V2(0) = 0; V2(1) = -1;
			}
		}
		else {
			VFDesign_SingularElement<T> *ele_singular = (VFDesign_SingularElement<T> *)ele;
			if (ele->type == 2) {//sink
				V0 = -V0; V1 = -V1; V2 = -V2;
			}
			else if (ele->type == 3) {////sandle
				V0(1) = -V0(1); V1(1) = -V1(1); V2(1) = -V2(1);
			}
			else if (ele->type == 4) {
				float t = V0[0];  V0[0] = -V0[1]; V0[1] = t;
				t = V1[0];  V1[0] = -V1[1]; V1[1] = t;
				t = V2[0];  V2[0] = -V2[1]; V2[1] = t;
			}
			else if (ele->type == 5) {
				float t = V0[1];  V0[1] = -V0[0]; V0[0] = t;
				t = V1[1];  V1[1] = -V1[0]; V1[0] = t;
				t = V2[1];  V2[1] = -V2[0]; V2[0] = t;
			}
		}
		vField->VF.row(v0) = V0; vField->VF.row(v1) = V1; vField->VF.row(v2) = V2;
		fixed_vertices.insert(v0); fixed_vertices.insert(v1); fixed_vertices.insert(v2);

	}
	void smooth() {
		//	return;
		vector<int> v_indices(vField->V.rows());
		int n_unknowns = build_variable_indices(v_indices, fixed_vertices, false);
		::smooth(*vField, v_indices, n_unknowns);
//		save_vectorfield_for_test(vField);
		VectorField2Image(vx,vy, glTex_img, *vField, image_width, image_height);
	}

	void update() {
		if (!vField) return;
	//	compute_vectors_for_elements();
		smooth();	
		VF = vField->VF;
		VF_Analysis(*vField, singularities);
	}
	void update_intermediate_data(){
		clear_intermediate_data();
		for (int s = 0; s < sketchs.size(); s++) {
			vector<int> triangles, nearest_point_indices;
			extract_sketch_triangle_strip(*vField, sketchs[s], triangles, nearest_point_indices);
//			triangles_on_sketchs[s] = triangles;
//			nearest_point_indices_to_triangles_on_sketchs[s] = nearest_point_indices;
			triangles_on_sketchs.push_back(triangles);
			nearest_point_indices_to_triangles_on_sketchs.push_back(nearest_point_indices);
		}
		compute_vectors_for_sketch_triangle_strip();
		compute_vectors_for_elements();
	}
	void clear() {
		sketchs.clear();
		for (auto ele : eles) {
			delete ele;
		}
		eles.clear();
		clear_intermediate_data();
	}

	void clear_intermediate_data() {		
		triangles_on_sketchs.clear();
		nearest_point_indices_to_triangles_on_sketchs.clear();
		nearest_k_points_indies_dists_to_triangleTrips_on_sketchs.clear();
		fixed_vertices.clear();
		if (VF.rows()) {
			VF.setZero(VF.rows(), VF.cols());
		}
		generate_noise_image();
	}

	public:
		std::vector<Singularity<T>> singularities;

		std::vector<unsigned char> noise_img, LIC_img, glTex_img;
		int image_width, image_height;
		void generate_noise_image() {
			const int size = image_width*image_height;
			noise_img.resize(size, 0);
			glTex_img.resize(size * 3, 0);

			gen_noise_img(&(noise_img[0]), image_width, image_height);
			gray2rgb(&(glTex_img[0]), &(noise_img[0]), image_width, image_height);
		}
public:

	//------------dispaly data------------
	void draw_sketchs(const int pointSize = 4) {
		glPointSize(pointSize);
		float *rgb = &(color_table[0]);
		glColor3f(rgb[0], rgb[1], rgb[2]);
		glBegin(GL_POINTS);
		for (int s = 0; s < sketchs.size(); s++) {
			Eigen::Matrix<T, Dynamic, Dynamic> &sketch =sketchs[s];
			if (current_select.type == 1 && current_select.ele_id == s)
				glColor3f(0, 0, 1);
			else
				glColor3f(rgb[0], rgb[1], rgb[2]);
			for (int i = 0; i < sketch.rows(); i++) {
				glVertex2f(sketch(i, 0), sketch(i, 1));
			}
		}		
		glEnd();
	}
	void draw_elements(const int pointSize = 8) {
		glPointSize(pointSize);		
		
		for (int i = 0; i < eles.size(); i++) {	
			float *rgb = &(color_table[3 * eles[i]->type]);
			if (current_select.type == 0 && current_select.ele_id == i) {
				glBegin(2*GL_POINTS);
				glColor3f(0, 0, 1);
			}
			else {
				glBegin(GL_POINTS);
				glColor3f(rgb[0], rgb[1], rgb[2]);
			}
			glVertex2f(eles[i]->p(0), eles[i]->p(1));			
		}
		glEnd();
	}
	void draw_triangles_on_sketchs(const std::vector<float> rgb = { 0,1.,0 }, const GLfloat thickness = 2.0,
		const T scale = 0.1)
	{
		glLineWidth(thickness);
		glPolygonMode(GL_FRONT, GL_LINE);
		glBegin(GL_TRIANGLES);

		for (int s = 0; s < triangles_on_sketchs.size(); s++) {
			vector<int> &triangles = triangles_on_sketchs[s];
			for (int i = 0; i < triangles.size(); i++) {
				int f = triangles[i];
				int nvertices = vField->F.cols();
				//	glBegin(GL_POLYGON);
				for (int j = 0; j < nvertices; j++) {
					int vert = vField->F(f, j);
					glVertex2f(vField->V(vert, 0), vField->V(vert, 1));
				}
				//glEnd();
			}
		}
		glEnd();
		glPolygonMode(GL_FRONT, GL_FILL);
	}
	//draw vectors of vertices on the triangletrip
	void draw_vectors_of_vertices_of_triangles_on_sketchs(const T scale = 0.1){		
		float *rgb = &(color_table[3]);
		glColor3f(rgb[0], rgb[1], rgb[2]);
		glBegin(GL_LINES);
		for (int s = sss; s < nearest_k_points_indies_dists_to_triangleTrips_on_sketchs.size(); s++) {
			const Eigen::Matrix<T, Dynamic, Dynamic> &sketch = sketchs[s];
			std::map<int, vector<std::pair<int, T>> > &nearest_k_points_indies_dists
				= nearest_k_points_indies_dists_to_triangleTrips_on_sketchs[s];
			for (const auto kv : nearest_k_points_indies_dists) {
				int vid = kv.first;
				Eigen::Matrix<T, 1, Dynamic> P = vField->V.row(vid);
				Eigen::Matrix<T, 1, Dynamic> V = vField->VF.row(vid);
				Eigen::Matrix<T, 1, Dynamic> Q = P + scale*V;
				glVertex2f(P(0), P(1));
				glVertex2f(Q(0), Q(1));
			}
		}
		glEnd();
		glLineWidth(1.0);
		return;
	}

	void draw_vectorfield_texture() {
		glEnable(GL_TEXTURE_2D);
		glShadeModel(GL_FLAT);

		// Display LIC image using texture mapping
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, image_width, image_height, 0,
			GL_RGB, GL_UNSIGNED_BYTE, &(glTex_img[0]));

		glBegin(GL_QUAD_STRIP);
		glTexCoord2f(0.0, 0.0); glVertex2f(0.0, 0.0);
		glTexCoord2f(0.0, 1.0); glVertex2f(0.0, 1.0);
		glTexCoord2f(1.0, 0.0); glVertex2f(1.0, 0.0);
		glTexCoord2f(1.0, 1.0); glVertex2f(1.0, 1.0);
		glEnd();
		glDisable(GL_TEXTURE_2D);
	}

	void draw_singularities() {
		float *rgb = &(color_table[3*2]);
		glColor3f(rgb[0], rgb[1], rgb[2]);

		glBegin(GL_POINTS);
		for (int i = 0; i < singularities.size(); i++) {
			Singularity<T> &singular = singularities[i];
			glColor3f(rgb[0], rgb[1], rgb[2]);
			glVertex2f(singular.pos[0], singular.pos[1]);
		}
		glEnd();
			
			
		for (int i = 0; i < singularities.size(); i++) {
			Singularity<T> &singular = singularities[i];
			if (singular.type != 3) continue;

			float x1, y1, x2, y2;
			float scale = 0.1, sV[2] = { scale*singular.eigenVectorL[0], scale*singular.eigenVectorL[1] };

			x1 = singular.pos[0] - sV[0]; y1 = singular.pos[1] - sV[1];
			x2 = singular.pos[0] + sV[0]; y2 = singular.pos[1] + sV[1];

			rgb = &(color_table[3 * 3]);
			glColor3f(rgb[0], rgb[1], rgb[2]);
			glBegin(GL_LINES);
			glVertex2f(x1,y1); glVertex2f(x2, y2);
			glEnd();

			scale = scale / 2;
			sV[0] = scale*singular.eigenVectorS[0]; sV[1] = scale*singular.eigenVectorS[1];

			x1 = singular.pos[0] - sV[0]; y1 = singular.pos[1] - sV[1];
			x2 = singular.pos[0] + sV[0]; y2 = singular.pos[1] + sV[1];
			rgb = &(color_table[3 * 4]);
			glColor3f(rgb[0], rgb[1], rgb[2]);
			glBegin(GL_LINES);
			glVertex2f(x1, y1); glVertex2f(x2, y2);
			glEnd();
		}
		
	}
	void draw() {
		draw_vectorfield_texture();
		draw_sketchs();
		draw_elements();
		if(show_analysis_)
			draw_singularities();
	}
	bool show_analysis_ = true;
	void show_analysis(bool show) { show_analysis_ = show; }

	//-----draw nearest k points of a vertex on the triangletrip----
	// ----just for debug------
	void draw_knn_points_from_a_vertex_on_triangletrip(const int triangleTrip_idx, const int vid, const int pointSize=5);

	//============edittting======================
	struct SelectElement {
		int x, y; T ox, oy, oz;
		int type = -1;
		int ele_id=-1, ele_sub_id = -1; // id for main selected element and the point id on the element if a curve selected
	}current_select;
	void select(const int x, const int y) {
		current_select.type = -1;
		GLdouble ox, oy, oz;
		viewport2object(x, y, ox, oy, oz);
		T eps = 0.15;		
		T min_dist = 2;
		int min_idx = -1;
		for (int i = 0; i < eles.size();i++) {
			VFDesign_Element<T>* ele = eles[i];
			Eigen::Matrix<T, 1, -1> o(1, ele->p.cols());
			o(0) = ox; o(1) = oy;
			Eigen::Matrix<T, 1, -1> op = ele->p - o;			
			T dist = op.norm();
			if (dist < min_dist) {
				min_dist = dist; min_idx = i;
			}			
		}
		if (min_dist < eps) {
			current_select.type = 0; current_select.ele_id = min_idx;
			current_select.x = x; current_select.y = y;
			current_select.ox = ox; current_select.oy = oy; current_select.oz = oz;
			return;
		}		
		vector<T> q = { (T)ox,(T)oy };
		for (int s = 0; s < sketchs.size(); s++) {
			const vector<int> &triangles = triangles_on_sketchs[s];
			for (int i = 0; i < triangles.size(); i++) {
				int f = triangles[i];
				vector<T> alpha(3);
				vField->GetBarycentricFacters(f, q, alpha);
				if ((alpha[0] >= 0 || fabs(alpha[0]) <= 1e-8) && alpha[0] <= 1
					&& (alpha[1] >= 0 || fabs(alpha[1]) <= 1e-8) && alpha[1] <= 1
					&& (alpha[2] >= 0 || fabs(alpha[2]) <= 1e-8) && alpha[2] <= 1)
				{
					current_select.type = 1; current_select.ele_id = s; current_select.ele_sub_id = i;
					current_select.x = x; current_select.y = y;
					current_select.ox = ox; current_select.oy = oy; current_select.oz = oz; 
					break;
				}
			}
		}
	}		
	int current_select_curve() {
		if (current_select.type == 1) return current_select.ele_id;
		return -1;
	}
	void del() { //delete current selected design element
		if (current_select.type == -1) return;
		else if (current_select.type == 0) {
			VFDesign_Element<T > *ele = *(eles.begin() + current_select.ele_id);
			eles.erase(eles.begin()+current_select.ele_id);
			current_select.type = -1;
			delete ele;
		}
		else {
			sketchs.erase(sketchs.begin() + current_select.ele_id);
			triangles_on_sketchs.erase(triangles_on_sketchs.begin() + current_select.ele_id);
			nearest_point_indices_to_triangles_on_sketchs.erase(nearest_point_indices_to_triangles_on_sketchs.begin() + current_select.ele_id);
			nearest_k_points_indies_dists_to_triangleTrips_on_sketchs.erase(nearest_k_points_indies_dists_to_triangleTrips_on_sketchs.begin() + current_select.ele_id);
//			nearest_point_indices_to_triangles_on_sketchs.clear();
//			nearest_k_points_indies_dists_to_triangleTrips_on_sketchs.clear();
			current_select.type = -1;
		}
		fixed_vertices.clear();
		compute_vectors_for_sketch_triangle_strip();
		compute_vectors_for_elements();
		update();
	}		
};

template<typename T>
void SingleFrameDesignData<T>::draw_knn_points_from_a_vertex_on_triangletrip(const int triangleTrip_idx, const int vid,
	const int pointSize) {
	glPointSize(pointSize);
	glBegin(GL_POINTS);

	const Eigen::Matrix<T, Dynamic, Dynamic> &sketch = sketchs[triangleTrip_idx];
	std::map<int, vector<std::pair<int, T>> > &nearest_k_points_indies_dists
		= nearest_k_points_indies_dists_to_triangleTrips_on_sketchs[triangleTrip_idx];
	std::map<int, vector<std::pair<int, T>> >::iterator it
		= nearest_k_points_indies_dists.begin();

	std::advance(it, vid);

	glColor3f(0, 0.5, 0.5);
	glVertex2f(vField->V(it->first, 0), vField->V(it->first, 1));
	for (int i = 0; i < it->second.size(); i++) {
		int idx = it->second[i].first;
		Eigen::Matrix<T, 1, Dynamic> P = sketch.row(idx);
		glColor3f(0, 0, 1);
		glVertex2f(P(0), P(1));
	}
	glEnd();
}

template<typename T>
class VectorFieldDesignData {
public:
	VectorFieldDesignData(const int img_width = 512, const int img_height = 512,
		const char *meshfile = 0, const char *design_file = 0) {
		init_vector_field(img_width, img_height, meshfile, design_file);
		//	image_width = img_width; image_height = img_height;		
		//	if (!design_file) {			generate_noise_image();		}

		SingleFrameDesignData<T> sfData(vField, image_width, image_height);
		SingleframeDesignDatas.push_back(sfData);
	}

	void init_vector_field(const int img_width = 512, const int img_height = 512,
		const char *mesh_file = 0, const char *design_file = "element.txt") {
		Eigen::MatrixXf V;	Eigen::MatrixXi F;
		//	grid2mesh(img_width, img_height, V, F);
		grid2mesh(mesh_grid_width, mesh_grid_height, V, F);
		vector<T> scale = { 1.f / (mesh_grid_width - 1) , 1.f / (mesh_grid_height - 1) };
		scaleV(V, scale);

		Eigen::Matrix<T, -1, -1> VF;
		vField = new VectorField<T>(V, F, VF);
		//vField = new VectorField<T>(V, F, VF, mesh_file);// "VTT.txt");
#if 0
		readDesignElements(eles, "element.txt");
		std::cout << "has read design elements\n";

		gen_vector_field_ByElements(*vField, eles);
		std::cout << "has generated vector field\n";
		updated_vector_field();
		std::cout << "has updated vector field\n";
#endif
	}

	void generate_noise_image() {
		const int size = image_width*image_height;
		noise_img.resize(size, 0);
		glTex_img.resize(size * 3, 0);

		gen_noise_img(&(noise_img[0]), image_width, image_height);
		gray2rgb(&(glTex_img[0]), &(noise_img[0]), image_width, image_height);
	}

	void add_singularity_element(const Eigen::Matrix<T, 1, -1> &pos, const int type,
		const T k = 1, const T d = 0.0004) {
		VFDesign_SingularElement<T>* singular = new VFDesign_SingularElement<T>(pos, d, type, k);
		SingleframeDesignDatas[current_sframeDesignData_idx].add_singularity_element(pos, type, k, d);
	}
	void add_regularity_element(const Eigen::Matrix<T, 1, -1> &pos, const Eigen::Matrix<T, 1, -1> &vec,
		const int type, const T d = 0.0004) {
		VFDesign_ResularElement<T> *resular = new VFDesign_ResularElement<T>(pos, d, type, vec);
		SingleframeDesignDatas[current_sframeDesignData_idx].add_regularity_element(pos, type, vec, d);
	}

	void updated_vector_field() {
		if (!vField || eles.size() == 0) return;
	}

	void isIn(vector<int> &in_vids, const std::vector<T> xs, const std::vector<T> &ys) {
		::isIn(vField->V, in_vids, xs, ys);
	}

	void add_sketch(const std::vector<T> xs, const std::vector<T> &ys) {
		if (SingleframeDesignDatas.size() > 0) {
			SingleframeDesignDatas[current_sframeDesignData_idx].add_sketch(xs, ys);
			return;
		}
	}
	
//	void compute_vectors_for_sketch_triangle_strip(const int sketch_idx);
//	void compute_vectors_for_sketch_triangle_strip();
//	void smooth();
	void draw_designed_vectorfield() { 
		SingleframeDesignDatas[current_sframeDesignData_idx].draw(); 
		if(show_mesh) draw_mesh_wireless();
	}
	void draw_interpolated_vectorfield() {
		if (interpolated_SingleframeDesignDatas.size() == 0) return;
		static int i = 0;	
		static int step = 0; 
		step++; if (step == 1000) step = 0;
		if (step == 0) {
			if (++i == interpolated_SingleframeDesignDatas.size()) i = 0;			
		}
		interpolated_SingleframeDesignDatas[i].draw();
	//	if (show_mesh) draw_mesh_wireless();
	}
	void draw_vectorfield() {
		if (show_design)draw_designed_vectorfield();
		else draw_interpolated_vectorfield();
	}

	void draw_mesh_wireless();
	void clear() {		}

	void clone_push() {
		SingleframeDesignDatas.push_back(SingleFrameDesignData<T>(
			SingleframeDesignDatas[SingleframeDesignDatas.size() - 1]));
		current_sframeDesignData_idx = SingleframeDesignDatas.size() - 1;
	}
	void select(const int x, const int y) {	
		SingleframeDesignDatas[current_sframeDesignData_idx].select(x,y);
	}
	void del() { //delete current selected design element
		SingleframeDesignDatas[current_sframeDesignData_idx].del();
	} 
	void show_analysis(bool show) { SingleframeDesignDatas[current_sframeDesignData_idx].show_analysis(show); }

	void interpolate(std::vector<SingleFrameDesignData<T>> &out_SingleframeDesignDatas,
		const int start_frame,const int end_frame,const std::vector<int> &curve_ids, const int num_t)
	{
		//resample the curves
		for (int j = 0; j < curve_ids.size(); j++) {
			int cid = curve_ids[j];			
			Eigen::Matrix<T, Dynamic, Dynamic> 	&start_curve = SingleframeDesignDatas[start_frame].sketchs[cid],
				&end_curve = SingleframeDesignDatas[end_frame].sketchs[cid], 
				start_curve_resample, end_curve_resample;
			int num = start_curve.rows() < end_curve.rows() ? start_curve.rows() : end_curve.rows();
			num--;
			resample_curve(start_curve_resample, start_curve,num);
			resample_curve(end_curve_resample, end_curve, num);
			SingleframeDesignDatas[start_frame].sketchs[cid] = start_curve_resample;
			SingleframeDesignDatas[end_frame].sketchs[cid] = end_curve_resample;
		}

		SingleFrameDesignData<T> &start_sfd = SingleframeDesignDatas[start_frame];
		SingleFrameDesignData<T> &end_sfd = SingleframeDesignDatas[end_frame];
		out_SingleframeDesignDatas.push_back(start_sfd);

		int n = num_t + 1;
		T step = 1. / n, t = step;
		for (int i = 1; i < n; i++, t += step) {
			SingleFrameDesignData<T> sfd = SingleFrameDesignData<T>(start_sfd);
			for (int j = 0; j < curve_ids.size(); j++) {
				int cid = curve_ids[j];
				Eigen::Matrix<T, Dynamic, Dynamic> curve,
					start_curve = start_sfd.sketchs[cid],
					end_curve = end_sfd.sketchs[cid];
				interpolate_a_curve(curve, start_curve, end_curve, t);
				sfd.sketchs[cid] = curve;
			}
			
			out_SingleframeDesignDatas.push_back(sfd);
		}
		out_SingleframeDesignDatas.push_back(end_sfd);

		for (int i = 0; i < out_SingleframeDesignDatas.size(); i++) {
			out_SingleframeDesignDatas[i].update_intermediate_data();
			out_SingleframeDesignDatas[i].update();
		}
	}

	void interpolate() {		
		if (start_frame < 0 || start_frame >= SingleframeDesignDatas.size()
			|| end_frame < 0 || end_frame >= SingleframeDesignDatas.size()) {
			std::cout << "start_frame: " << start_frame << "end_frame:" << end_frame 
				<< "SingleframeDesignDatas.size():" << SingleframeDesignDatas.size() << "\n";
			return;
		}
		int num_t = 10;
//		std::vector<SingleFrameDesignData<T>> out_SingleframeDesignDatas;
		interpolate(interpolated_SingleframeDesignDatas,start_frame, end_frame, interpolate_curves_ids, num_t);
 //  	    interpolated_SingleframeDesignDatas = out_SingleframeDesignDatas;
		show_design = false;		
	}
	void add_interpolate_curve() {
		int cid = SingleframeDesignDatas[current_sframeDesignData_idx].current_select_curve();
		if (cid >= 0) interpolate_curves_ids.push_back(cid);
	}
public:
	bool show_design = true;
	std::vector<int> interpolate_curves_ids;
	int start_frame = 0, end_frame = 1;
	std::vector<SingleFrameDesignData<T>> interpolated_SingleframeDesignDatas;

	std::vector<SingleFrameDesignData<T>> SingleframeDesignDatas;

	int current_sframeDesignData_idx = 0;
	VectorField<T> *vField = 0;	
	int image_width = 512, image_height = 512;
	int mesh_grid_width = 16, mesh_grid_height = 16;	
	bool show_mesh = false;
};



template<typename T>
void VectorFieldDesignData<T>::draw_mesh_wireless(){
	glColor3f(0.0, 0., 0.);
	GLfloat thickness = 2.0;
	glLineWidth(thickness);
	glPolygonMode(GL_FRONT, GL_LINE);

	glBegin(GL_TRIANGLES);
	int nfaces = vField->F.rows();
	for (int f = 0; f<nfaces; f++) {		
		int nvertices = vField->F.cols();
		//	glBegin(GL_POLYGON);
		for (int j = 0; j<nvertices; j++)		{
			int vert = vField->F(f, j);
			glVertex2f(vField->V(vert, 0), vField->V(vert, 1));		
		}
		//glEnd();
	}
	glEnd();	
	glPolygonMode(GL_FRONT, GL_FILL);
	glLineWidth(1.0);
}



