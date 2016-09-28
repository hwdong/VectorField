#include "VectorField.h"

template<typename T>
class VFDesign_Element {
public:
	Eigen::Matrix<T, 1, -1> p;
	int type; T d;
public:
	VFDesign_Element(Eigen::Matrix<T, 1, -1> p0,T d_,int type_) :p(p0),d(d_),type(type_) { }
	virtual Eigen::Matrix<T, 1, -1>  operator()(VectorField<T> &VF, const Eigen::Matrix<T, 1, -1> &p) = 0;
};

template<typename T>
class VFDesign_SingularElement :VFDesign_Element<T>
{
	T k;
public:
	VFDesign_SingularElement(const Eigen::Matrix<T, 1, -1> &p, T d, int type, T k_) :k(k_), VFDesign_Element(p,d,type) {}
	virtual Eigen::Matrix<T, 1, -1>  operator()(VectorField<T> &VF, const Eigen::Matrix<T, 1, -1> &p)  {
		const int dim = VF.V.cols();
		if (dim == 2) {
			Eigen::Matrix<T, 1, -1>  p0p = p - this->p;
			T length2 = p0p*p0p.adjoint();

			Eigen::Matrix<T, 1, -1>  V = k*exp(-d*length2)*p0p;
			if (type == 2)   //Sink
				V = -V;
			else if (type == 3)  //sandle
				V[1] = -V[1];

			else if (type == 4) { //counter-clockwisecenter
				float t = V[0];
				V[0] = -V[1];
				V[1] = t;
			}
			else if (type == 5) { //clockwisecenter
				float t = V[1];
				V[1] = -V[0];
				V[0] = t;
			}
			return V;
		}
		else {
		}
	};	
};


template<typename T>
class VFDesign_ResularElement :VFDesign_Element<T>
{
	Eigen::Matrix<T, 1, -1> v;
public:
	VFDesign_ResularElement(const Eigen::Matrix<T, 1, -1>& p, float d, int type, const Eigen::Matrix<T, 1, -1>& v_)
		: v(v_), VFDesign_Element(p,d,type) {	}
	virtual Eigen::Matrix<T, 1, -1>  operator()(VectorField<T> &VF, const Eigen::Matrix<T, 1, -1> &p)  {
		const int dim = VF.V.cols();
		if (dim == 2) {
			Eigen::Matrix<T, 1, -1>  p0p = p - this->p;
			T length2 = p0p*p0p.adjoint();
			Eigen::Matrix<T, 1, -1> V = exp(-d*length2)*v;
			return V;
		}
		else {

		}
	};
};

template<typename T>
void gen_vector_field_ByElements(VectorField<T> &VF, std::vector<VFDesign_Element<T>*> eles, bool normlizeF = false)
{
	const int dim = VF.V.cols();
	if (dim == 2) {
		for (int i = 0; i < VF.V.rows(); i++) {
			Eigen::Matrix<T, 1, 2> p = VF.V.row(i);
			Eigen::Matrix<T, 1, 2> V(0, 0);
			for (auto ele : eles) {
				Eigen::Matrix<T, 1, 2> t = (*ele)(VF,p);
				V += t;
			}
			VF.VF.row(i) = V;
			
		}
	}
	else {//3d surface 

	}	
}

template<typename T>
bool readDesignElements(std::vector<VFDesign_Element<T>*> &eles, const char *filename)
{
	std::ifstream iF(filename);
	if (!iF) return false;
	int n;
	iF >> n;
	eles.resize(n);
	int type;  T d, k;
	
	Eigen::Matrix<T, 1, -1> pos(1, 2),vec(1,2);
	VFDesign_Element<T>* element = 0;
	for (int i = 0; i < n; i++) {
		iF >> type;
		if (type == 0) { //regular
			iF >> pos(0) >> pos(1) >> d >> vec(0) >> vec(1);
			VFDesign_ResularElement<T> *resular = new VFDesign_ResularElement<T>(pos, d, type, vec);
			eles[i] = (VFDesign_Element<T>*)resular;
		}
		else {
			iF >> pos(0) >> pos(1) >> d >> k;
			VFDesign_SingularElement<T>* singular = new VFDesign_SingularElement<T>(pos, d, type, k);
			eles[i] = (VFDesign_Element<T>*)singular;
		}
		
	}
}



#include "Analysis.h"
#include <fstream>
template<typename T>
void test_gen_vector_field_ByElements(VectorField<T> *&vField
	,const int width,const int height ) 
{
//	const int  width = 30, height = 30;
	Eigen::MatrixXf V;	Eigen::MatrixXi F;
	grid2mesh(width, height, V, F);
	Eigen::Matrix<T, -1, -1> VF;
	vField = new VectorField<T>(V, F, VF); 
//	vField = new VectorField<T>(V, F, VF, "VTT.txt");

	std::vector<VFDesign_Element<T>*> eles;

	Eigen::Matrix<T, 1, -1> p0(1,2); 
	p0(0) = width / 4; p0(1) =  height / 2;

	T d = 10. / (width*height), k = 1;
	//	float d = 0.0001 , k = 1.;
	int type = 1; VFDesign_SingularElement<T> *singular = 0;
#if 1		
	singular = new VFDesign_SingularElement<T>(p0, d, type,k);
	eles.push_back((VFDesign_Element<T>*)singular);
#endif

#if 1	
	type = 2;
	p0[0] = 3 * width / 4; p0[1] = 1 * height / 2; //k = 0.5;
	singular = new VFDesign_SingularElement<T>(p0, d, type, k);
	eles.push_back((VFDesign_Element<T>*)singular);
#endif	
#if 1	
	type = 4;
	p0[0] = 1 * width / 2; p0[1] = 3 * height / 4; //k = 0.5;
	singular = new VFDesign_SingularElement<T>(p0, d, type, k);
	eles.push_back((VFDesign_Element<T>*)singular);
#endif	
#if 1	
	type = 0;
	Eigen::Matrix<T, 1, -1> v0(1, 2);
	v0(0) = 1; v0(1) = 0;
	VFDesign_ResularElement<T> *regular = new VFDesign_ResularElement<T>(p0, d, type, v0);
	eles.push_back((VFDesign_Element<T>*)regular);
#endif
	gen_vector_field_ByElements(*vField,eles);

#if 0
	std::vector<Singularity<T>> singularities;
	VF_Analysis(*vField, singularities);
#endif

#if 0
	std::ofstream oF("D:\\python_ex\\points.txt");
	for (int i = 0; i < V.rows(); i++) {
		oF << V(i, 0) << " " << V(i, 1) << " " 
			<< vField->VF(i, 0) << " " << vField->VF(i, 1) << "\n";
	}
	oF.close();

	std::ofstream oFt("D:\\python_ex\\triangles.txt");
	for (int i = 0; i < F.rows(); i++) {
		oFt << F(i, 0) << " " << F(i, 1) << " " << F(i, 2) << "\n";
	}
	oFt.close();

	std::ofstream oF2("D:\\python_ex\\trajectory.txt");
	for (int i = 0; i < singularities.size(); i++) {
		//	std::cout << trajectory[i](0) << " " << trajectory[i](1) << "\n";
		oF2 << singularities[i].pos[0] << " " << singularities[i].pos[1] << "\n";
	}
	oF2.close();
#endif

}


//   opengl 2d mouse real coordinates
//http://stackoverflow.com/questions/9123648/opengl-2d-mouse-click-location