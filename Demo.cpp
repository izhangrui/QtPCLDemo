#include "Demo.h"

PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
PointCloudT::Ptr cloud_a(new PointCloudT);
PointCloudT::Ptr cloud_b(new PointCloudT);
PointCloudT::Ptr cloud_c(new PointCloudT);
PointCloudT::Ptr cloud_d(new PointCloudT);
PointCloudT::Ptr cloud_e(new PointCloudT);
PointCloudT::Ptr cloud_x(new PointCloudT);
PointCloudT::Ptr cloud_a_nei(new PointCloudT);
PointCloudT::Ptr cloud_a_wai(new PointCloudT);
PointCloudT::Ptr cloud_b_nei(new PointCloudT);
PointCloudT::Ptr cloud_b_wai(new PointCloudT);
PointCloudT::Ptr cloud_c_nei(new PointCloudT);
PointCloudT::Ptr cloud_c_wai(new PointCloudT);
PointCloudT::Ptr cloud_d_nei(new PointCloudT);
PointCloudT::Ptr cloud_d_wai(new PointCloudT);

Demo::Demo(QWidget *parent)
	: QMainWindow(parent)
{
	vtkObject::GlobalWarningDisplayOff();
	ui.setupUi(this);
	connect(ui.pushButton_openfile, SIGNAL(clicked()), this, SLOT(open_file()));
	connect(ui.pushButton_m_radar1, SIGNAL(clicked()), this, SLOT(m_radar1()));
	connect(ui.pushButton_m_servo, SIGNAL(clicked()), this, SLOT(m_servo()));
}

void Demo::open_file()
{
	//circle_3d();
	QString fileName = QFileDialog::getOpenFileName(
		this, tr("open image file"),
		"./", tr("Image files(*.pcd *.obj *.ply *.pgm *.png *.ppm *.xbm *.xpm);;All files (*.*)"));

	if (fileName.isEmpty())
	{
		QMessageBox mesg;
		mesg.warning(this, "警告", "打开图片失败!");
		return;
	}
	else
	{
		std::string sfileName = fileName.toStdString();
		//pcl::io::loadPCDFile(sfileName, *cloud);
		//std::string sfile1(CW2A(file1.GetString()));
		string suffixStr = sfileName.substr(sfileName.find_last_of('.') + 1);//获取文件后缀 
		std::string extply("ply");
		std::string extpcd("pcd");
		std::string extobj("obj");
		if (!((suffixStr == extply) || (suffixStr == extpcd) || (suffixStr == extobj))) {
			std::cout << "文件格式不支持!" << std::endl;
			std::cout << "支持文件格式：*.pcd和*.ply！" << std::endl;
			return;
		}

		//根据文件格式选择输入方式
		//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>); //创建点云对象指针，用于存储输入
		if (suffixStr == extply) {
			if (pcl::io::loadPLYFile(sfileName, *cloud) == -1) {
				cout << "Could not read ply file!\n";
				return;
			}
		}
		else if (suffixStr == extobj) {
			pcl::PolygonMesh mesh;
			if (pcl::io::loadPolygonFileOBJ(sfileName, mesh) == -1) {
				cout << "Could not read obj file!\n";
				return;
			}
			else
			{
				pcl::fromPCLPointCloud2(mesh.cloud, *cloud);
				//pcl::io::savePCDFileASCII(sfileName + ".pcd", *cloud);
			}
		}
		else {
			if (pcl::io::loadPCDFile(sfileName, *cloud) == -1) {
				cout << "Could not read pcd file!\n";
				return;
			}
		}
		////cout << *cloud << endl;
		//*cloud_red = *cloud_null;
		//if (cloud_x->points.size() > 0)
		//{
		//	*cloud_yellow = *cloud_x;
		//}
		////cout << cloud_red->points.size()<<endl;
		//resultfile << sfileName << endl;
		show_cloud();
	}
}

void Demo::show_cloud()
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	ui.qvtkWidget->SetRenderWindow(viewer->getRenderWindow());
	ui.qvtkWidget->update();
	pcl::visualization::PointCloudColorHandlerCustom<PointXYZ>cloud_yellow(cloud, 255, 255, 0);
	viewer->addPointCloud(cloud, cloud_yellow, "cloud_color1");  
	viewer->spin();
	//cloud_mutex.unlock();
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
}

void Demo::m_radar1()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_outr(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_outl(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_view(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_base(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_1(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_0(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_i(new pcl::PointCloud<pcl::PointXYZ>);
	//*cloud_yellow = *cloud_null;
	*cloud_in = *cloud;
	float measure_1sl_z = 12, measure_1sl_y = 127.5, measure_1sl_x = 415,
		measure_1sr_z = measure_1sl_z + 10,
		measure_1sr_y = measure_1sl_y - 80,
		measure_1sr_x = measure_1sl_x;


	//（第0块定位）
	float measure_01l_z = measure_1sl_z + 20.7, measure_01l_y = measure_1sl_y - 15, measure_01l_x = measure_1sl_x;
	float measure_01r_z = measure_01l_z + 7.5, measure_01r_y = measure_01l_y - 50, measure_01r_x = measure_1sl_x;

	float measure_02l_z = measure_01l_z - 126, measure_02l_y = measure_01l_y, measure_02l_x = measure_1sl_x;
	float measure_02r_z = measure_02l_z + 7.5, measure_02r_y = measure_01r_y, measure_02r_x = measure_1sl_x;



	//第01块区域直通滤波l
	passtTrougH(cloud_in, cloud_outl,
		measure_01l_x, measure_01l_x + 10,
		measure_01l_y - 10, measure_01l_y,
		measure_01l_z, measure_01l_z + 5);

	*cloud_view = *cloud_outl + *cloud_view;


	//第01块区域直通滤波r
	passtTrougH(cloud_in, cloud_outr,
		measure_01r_x, measure_01r_x + 10,
		measure_01r_y - 10, measure_01r_y,
		measure_01r_z, measure_01r_z + 5);
	*cloud_view = *cloud_outr + *cloud_view;



	//第02块区域直通滤波l
	passtTrougH(cloud_in, cloud_outl,
		measure_02l_x, measure_02l_x + 10,
		measure_02l_y - 10, measure_02l_y,
		measure_02l_z, measure_02l_z + 5);


	*cloud_view = *cloud_outl + *cloud_view;


	//第02块区域直通滤波r
	passtTrougH(cloud_in, cloud_outr,
		measure_02r_x, measure_02r_x + 10,
		measure_02r_y - 10, measure_02r_y,
		measure_02r_z, measure_02r_z + 5);

	*cloud_view = *cloud_outr + *cloud_view;
	filterMean(cloud_view, cloud_view);
	//yizhicaiyang(cloud_view, cloud_view);
	*cloud_0 = *cloud_view;
	pcl::io::savePCDFile("cloud_0.pcd", *cloud_view);
	cout << "0到0:" << endl;
	pmd(cloud_0, cloud_0);

	//第0块计算结束


	//读取点云文件

	//第1块区域直通滤波l
	passtTrougH(cloud_in, cloud_outl,
		measure_1sl_x, measure_1sl_x + 10,
		measure_1sl_y - 15, measure_1sl_y,
		measure_1sl_z, measure_1sl_z + 5);



	//第1块区域直通滤波r
	passtTrougH(cloud_in, cloud_outr,
		measure_1sr_x, measure_1sr_x + 10,
		measure_1sr_y - 15, measure_1sr_y,
		measure_1sr_z, measure_1sr_z + 5);

	//pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_tr_color_3(cloud_out, 0, 255, 0);
	//viewer->addPointCloud(cloud_out, cloud_tr_color_3, "cloud_tr_color_3");
	*cloud_1 = *cloud_outr + *cloud_outl;
	filterMean(cloud_1, cloud_1);
	//yizhicaiyang(cloud_1, cloud_1);
	if (cloud_1->points.size() < 1000)
	{
		cout << "cloud1 is empty" << endl;
	}
	else
	{
		//cout << "the Flatness of 1 to 0 is: " ;
		//pmd(cloud_1, cloud_view);
		cout << "1到0:" << endl;
		pmd(cloud_1, cloud_0);
		cout << "1到1:" << endl;
		pmd(cloud_1, cloud_1);

		//cout << "\n";
		*cloud_view = *cloud_1 + *cloud_view;
		pcl::io::savePCDFile("cloud_1.pcd", *cloud_1);
	}
	//cout << "cloud_1平面度是" << endl;

	for (int i = 2; i < 7; i++)
	{
		string str;
		stringstream exchange1;
		exchange1 << i;
		exchange1 >> str;

		float measure_2sl_z = measure_1sl_z - 17.7 * (i - 1), measure_2sr_z = measure_2sl_z + 10,
			measure_2sl_x = 415, measure_2sr_x = measure_2sl_x,
			measure_2sr_y = measure_1sr_y, measure_2sl_y = measure_1sl_y;


		//第i块区域直通滤波l
		passtTrougH(cloud_in, cloud_outl,
			measure_2sl_x, measure_2sl_x + 10,
			measure_2sl_y - 15, measure_2sl_y,
			measure_2sl_z, measure_2sl_z + 5);

		string ll = "cloud_" + str;


		//第i块区域直通滤波r
		passtTrougH(cloud_in, cloud_outr,
			measure_2sr_x, measure_2sr_x + 10,
			measure_2sr_y - 15, measure_2sr_y,
			measure_2sr_z, measure_2sr_z + 5);
		*cloud_i = *cloud_outr + *cloud_outl;
		filterMean(cloud_i, cloud_i);
		//yizhicaiyang(cloud_i, cloud_i);
		if (cloud_i->points.size() < 1000)
		{
			cout << "cloud" + str + " is empty" << endl;
		}
		else
		{
			//cout << "the Flatness of cloud" + str + " to base cloud is: ";
			//pmd(cloud_i, cloud_view);
			cout << str +"到0"<< endl;;
			pmd(cloud_i, cloud_0);
			cout << str + "到" + str << endl;;
			pmd(cloud_i, cloud_i);
			//cout << "\n";
			*cloud_view = *cloud_i + *cloud_view;
			pcl::io::savePCDFile(ll + ".pcd", *cloud_i);
		}
	}
	cout << "所有到0" << endl;
	pmd(cloud_view, cloud_0);

	cout << "========" << endl;
	for (int i = 0; i < 7; i++)
	{
		string str;
		stringstream exchange1;
		exchange1 << i;
		exchange1 >> str;
		pcl::io::loadPCDFile("cloud_" + str + ".pcd", *cloud_i);
		cout << str+"所有" << endl;
		pmd(cloud_i, cloud_view);
		/*cout << "the Flatness of cloud" + str + "to itself is: ";
		pmd(cloud_i, cloud_i);*/
	}
	cout << "所有到所有" << endl;
	pmd(cloud_view, cloud_view);
	pcl::io::savePCDFile("cloud_sum.pcd", *cloud_view);
}

void Demo::m_servo()
{
	if (cloud->points.size() == 0)
		return;
	//读取基准模型
	if (cloud_a->points.size() == 0)
	{
		pcl::PolygonMesh mesh;
		if (pcl::io::loadPolygonFileOBJ("./model/model.obj", mesh) == -1) {
			cout << "Could not read ply file!\n";
			return;
		}
		else
		{
			pcl::fromPCLPointCloud2(mesh.cloud, *cloud_x);//模型5
			//pcl::io::savePCDFileASCII(sfileName + ".pcd", *cloud);
		}
		if (pcl::io::loadPolygonFileOBJ("./model/plane_a.obj", mesh) == -1) {
			cout << "Could not read ply file!\n";
			return;
		}
		else
		{
			pcl::fromPCLPointCloud2(mesh.cloud, *cloud_a);
			//pcl::io::savePCDFileASCII(sfileName + ".pcd", *cloud);
		}
		if (pcl::io::loadPolygonFileOBJ("./model/plane_b.obj", mesh) == -1) {
			cout << "Could not read ply file!\n";
			return;
		}
		else
		{
			pcl::fromPCLPointCloud2(mesh.cloud, *cloud_b);
			//pcl::io::savePCDFileASCII(sfileName + ".pcd", *cloud);
		}
		if (pcl::io::loadPolygonFileOBJ("./model/plane_c.obj", mesh) == -1) {
			cout << "Could not read ply file!\n";
			return;
		}
		else
		{
			pcl::fromPCLPointCloud2(mesh.cloud, *cloud_c);
			//pcl::io::savePCDFileASCII(sfileName + ".pcd", *cloud);
		}
		if (pcl::io::loadPolygonFileOBJ("./model/plane_d.obj", mesh) == -1) {
			cout << "Could not read ply file!\n";
			return;
		}
		else
		{
			pcl::fromPCLPointCloud2(mesh.cloud, *cloud_d);
			//pcl::io::savePCDFileASCII(sfileName + ".pcd", *cloud);
		}
		if (pcl::io::loadPolygonFileOBJ("./model/plane_e.obj", mesh) == -1) {
			cout << "Could not read ply file!\n";
			return;
		}
		else
		{
			pcl::fromPCLPointCloud2(mesh.cloud, *cloud_e);
			//pcl::io::savePCDFileASCII(sfileName + ".pcd", *cloud);
		}
		if (pcl::io::loadPCDFile("./model/board_a_inside.pcd", *cloud_a_nei) == -1) {
			cout << "Could not read pcd file!\n";
			return;
		}
		if (pcl::io::loadPCDFile("./model/board_a_outside.pcd", *cloud_a_wai) == -1) {
			cout << "Could not read pcd file!\n";
			return;
		}
		if (pcl::io::loadPCDFile("./model/board_b_inside.pcd", *cloud_b_nei) == -1) {
			cout << "Could not read pcd file!\n";
			return;
		}
		if (pcl::io::loadPCDFile("./model/board_b_outside.pcd", *cloud_b_wai) == -1) {
			cout << "Could not read pcd file!\n";
			return;
		}
		if (pcl::io::loadPCDFile("./model/board_c_inside.pcd", *cloud_c_nei) == -1) {
			cout << "Could not read pcd file!\n";
			return;
		}
		if (pcl::io::loadPCDFile("./model/board_c_outside.pcd", *cloud_c_wai) == -1) {
			cout << "Could not read pcd file!\n";
			return;
		}
		if (pcl::io::loadPCDFile("./model/board_d_inside.pcd", *cloud_d_nei) == -1) {
			cout << "Could not read pcd file!\n";
			return;
		}
		if (pcl::io::loadPCDFile("./model/board_d_outside.pcd", *cloud_d_wai) == -1) {
			cout << "Could not read pcd file!\n";
			return;
		}
		cout << "initial measured part\n";
	}
	Eigen::Matrix4d transMatrix;
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::ModelCoefficients::Ptr a_n(new pcl::ModelCoefficients);
	pcl::ModelCoefficients::Ptr b_n(new pcl::ModelCoefficients);
	pcl::ModelCoefficients::Ptr c_n(new pcl::ModelCoefficients);
	pcl::ModelCoefficients::Ptr d_n(new pcl::ModelCoefficients);
	pcl::ModelCoefficients::Ptr a_w(new pcl::ModelCoefficients);
	pcl::ModelCoefficients::Ptr b_w(new pcl::ModelCoefficients);
	pcl::ModelCoefficients::Ptr c_w(new pcl::ModelCoefficients);
	pcl::ModelCoefficients::Ptr d_w(new pcl::ModelCoefficients);
	//匹配定位
	icpMatch(cloud, cloud_x, transMatrix);
	PointCloudT::Ptr cloud_temp(new PointCloudT);
	transformPointCloud(*cloud, *cloud_temp, transMatrix);
	//邻近分割
	cout << "a,b,c,d四个部分内圆外圆的参数分别为:" << endl;
	measure_part(cloud_temp, cloud_a, cloud_a_nei, cloud_a_wai, a_n, a_w);
	measure_part(cloud_temp, cloud_b, cloud_b_nei, cloud_b_wai, b_n, b_w);
	measure_part(cloud_temp, cloud_c, cloud_c_nei, cloud_c_wai, c_n, c_w);
	measure_part(cloud_temp, cloud_d, cloud_d_nei, cloud_d_wai, d_n, d_w);
	//计算法线
	double ccenter[8][3];
	ccenter[0][0] = a_n->values[0]; ccenter[0][1] = a_n->values[1]; ccenter[0][2] = a_n->values[2];
	ccenter[1][0] = a_w->values[0]; ccenter[1][1] = a_w->values[1]; ccenter[1][2] = a_w->values[2];
	ccenter[2][0] = b_n->values[0]; ccenter[2][1] = b_n->values[1]; ccenter[2][2] = b_n->values[2];
	ccenter[3][0] = b_w->values[0]; ccenter[3][1] = b_w->values[1]; ccenter[3][2] = b_w->values[2];
	ccenter[4][0] = c_n->values[0]; ccenter[4][1] = c_n->values[1]; ccenter[4][2] = c_n->values[2];
	ccenter[5][0] = c_w->values[0]; ccenter[5][1] = c_w->values[1]; ccenter[5][2] = c_w->values[2];
	ccenter[6][0] = d_n->values[0]; ccenter[6][1] = d_n->values[1]; ccenter[6][2] = d_n->values[2];
	ccenter[7][0] = d_w->values[0]; ccenter[7][1] = d_w->values[1]; ccenter[7][2] = d_w->values[2];
	double cnormal[8][3];
	int count = 0;
	for(int i=0;i<2;i++)
		for (int j = 2; j < 4; j++)
		{
			for (int k = 0; k < 3; k++)
			{
				cnormal[count][k] = ccenter[i][k] - ccenter[j][k];
			}
			count++;
		}
	for (int i = 4; i<6; i++)
		for (int j = 6; j < 8; j++)
		{
			for (int k = 0; k < 3; k++)
			{
				cnormal[count][k] = ccenter[i][k] - ccenter[j][k];
			}
			count++;
		}
	cout << "8条法向:" << endl;
	for (int i = 0; i < 8; i++)
	{
		for (int j = 0; j < 3; j++)
			cout << cnormal[i][j] << " ";
		cout << "\n";
	}
	//计算夹角
	double cangle[16];
	double angle_sum = 0.0;
	count = 0;
	cout << "由8条法向计算出来的16个夹角:" << endl;
	for(int i=0;i<4;i++)
		for (int j = 4; j < 8; j++)
		{
			cangle[count] = v2angle(cnormal[i][0], cnormal[i][1], cnormal[i][2], cnormal[j][0], cnormal[j][1], cnormal[j][2]);
			angle_sum += cangle[count];
			cout << cangle[count]<< endl;
			count++;
		}
	cout << "夹角的均值:" << angle_sum / count << endl;
	double cerror=0.0;
	double cerror_sum = 0.0;
	for (int i=0; i < 16; i++)
	{
		cerror = cangle[i] - angle_sum / count;
		cerror_sum += cerror*cerror;
	}
	cout << "夹角的标准差:" << sqrt(cerror_sum) / count << endl;
}

