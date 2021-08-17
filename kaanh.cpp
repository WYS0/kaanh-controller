﻿#include <algorithm>
#include "kaanh.h"

using namespace aris::dynamic;
using namespace aris::plan;


namespace kaanh
{
    //configuring controller
    auto createControllerEXO()->std::unique_ptr<aris::control::Controller>
	{
		std::unique_ptr<aris::control::Controller> controller(new aris::control::EthercatController);

        for (aris::Size i = 0; i < 3; ++i)//3 电机
		{
#ifdef WIN32
            double pos_offset[3]
			{
                0,0,0
			};
#endif

            double pos_offset[3]
            {
                0,   0,   0
			};

    //        double pos_factor[6]
    //		{//2^17=131072  17 位编码器 减速比81？
    //            131072.0 * 81 / 2 / PI, 131072.0 * 81 / 2 / PI, 131072.0 * 81 / 2 / PI, 131072.0 * 72.857 / 2 / PI, 131072.0 * 81 / 2 / PI, 131072.0 * 50 / 2 / PI
    //	};
            double pos_factor[6]
            {
                4*2048.0* 100 *4 / 2 / PI, 4*2048.0* 100 / 2 / PI, 4*2048.0* 100 / 2 / PI   };//4倍频？ 比例对了
            double max_pos[6]
			{
               70.0 / 360 * 2 * PI, 80.0 / 360 * 2 * PI,	130.0 / 360 * 2 * PI,
			};
            double min_pos[6]
			{
                -1.0 / 360 * 2 * PI, -1.0 / 360 * 2 * PI, -1.0/ 360 * 2 * PI,
			};
            double max_vel[6]
			{
                60.0 / 360 * 2 * PI, 60.0 / 360 * 2 * PI, 90.0 / 360 * 2 * PI,
			};
            double max_acc[6]
			{
                8000.0 / 360 * 2 * PI, 8000.0 / 360 * 2 * PI, 10000.0 / 360 * 2 * PI,
			};

			std::string xml_str =
                "<EthercatMotion phy_id=\"" + std::to_string(i) + "\" product_code=\"0x00030924\""
                " vendor_id=\"0x0000009a\" revision_num=\"0x00010420\" dc_assign_activate=\"0x0300\""
				" min_pos=\"" + std::to_string(min_pos[i]) + "\" max_pos=\"" + std::to_string(max_pos[i]) + "\" max_vel=\"" + std::to_string(max_vel[i]) + "\" min_vel=\"" + std::to_string(-max_vel[i]) + "\""
				" max_acc=\"" + std::to_string(max_acc[i]) + "\" min_acc=\"" + std::to_string(-max_acc[i]) + "\" max_pos_following_error=\"0.10000000000000001\" max_vel_following_error=\"0.5\""
				" home_pos=\"0\" pos_factor=\"" + std::to_string(pos_factor[i]) + "\" pos_offset=\"" + std::to_string(pos_offset[i]) + "\">"
                "	<SyncManagerPoolObject name=\"sm_pool\">"
                "		<SyncManager name=\"sm\" is_tx=\"false\"/>"
                "		<SyncManager name=\"sm\" is_tx=\"true\"/>"
                "		<SyncManager name=\"sm\" is_tx=\"false\">"
                "			<Pdo name=\"pdo\" index=\"0x1605\">"
                "               <PdoEntry name=\"Target Position\" index=\"0x607a\" subindex=\"0x00\" size=\"32\"/>"
                "				<PdoEntry name=\"Target Velocity\" index=\"0x60ff\" subindex=\"0x00\" size=\"32\"/>"
                "				<PdoEntry name=\"Target Torque\" index=\"0x6071\" subindex=\"0x00\" size=\"16\"/>"
                "				<PdoEntry name=\"Max. Torque\" index=\"0x6072\" subindex=\"0x00\" size=\"16\"/>"
                "				<PdoEntry name=\"Control word\" index=\"0x6040\" subindex=\"0x00\" size=\"16\"/>"
                "				<PdoEntry name=\"Mode of operation\" index=\"0x6060\" subindex=\"0x00\" size=\"8\"/>"
				"			</Pdo>"
				"		</SyncManager>"
                "		<SyncManager name=\"sm\" is_tx=\"true\">"
                "			<Pdo name=\"pdo\" index=\"0x1a04\">"
                "				<PdoEntry name=\"Position actual value\" index=\"0x6064\" subindex=\"0x00\" size=\"32\"/>"
                "				<PdoEntry name=\"Position Following error actual value\" index=\"0x60f4\" subindex=\"0x00\" size=\"32\"/>"
                "				<PdoEntry name=\"Torque actual value\" index=\"0x6077\" subindex=\"0x00\" size=\"16\"/>"
                "				<PdoEntry name=\"Status word\" index=\"0x6041\" subindex=\"0x00\" size=\"16\"/>"
                "				<PdoEntry name=\"Mode of operation display\" index=\"0x6061\" subindex=\"0x00\" size=\"8\"/>"
                "			</Pdo>"
				"		</SyncManager>"
                "	</SyncManagerPoolObject>"
				"</EthercatMotion>";


            //第1个节点：外展电机//第2个节点：前屈电机//第3个节点：屈肘电机
            controller->slavePool().add<aris::control::EthercatMotion>().loadXmlStr(xml_str);
            controller->slavePool().back().setPhyId((std::uint16_t&)i);
//我注释到
            //dynamic_cast<aris::control::EthercatSlave&>(controller->slavePool().back()).scanInfoForCurrentSlave();
            //dynamic_cast<aris::control::EthercatSlave&>(controller->slavePool().back()).scanPdoForCurrentSlave();
            //dynamic_cast<aris::control::EthercatSlave&>(controller->slavePool().back()).setDcAssignActivate(0x300);

        }

			//添加第4个EtherCAT Node：采集设备基站
			controller->slavePool().add<aris::control::EthercatSlave>();
            controller->slavePool().back().setPhyId(3);
			dynamic_cast<aris::control::EthercatSlave&>(controller->slavePool().back()).scanInfoForCurrentSlave();
			dynamic_cast<aris::control::EthercatSlave&>(controller->slavePool().back()).scanPdoForCurrentSlave();
			//添加第5个EtherCAT Node：5V——DAC
            controller->slavePool().add<aris::control::EthercatSlave>();
            controller->slavePool().back().setPhyId(4);
            dynamic_cast<aris::control::EthercatSlave&>(controller->slavePool().back()).scanInfoForCurrentSlave();
            dynamic_cast<aris::control::EthercatSlave&>(controller->slavePool().back()).scanPdoForCurrentSlave();
			//添加第6个EtherCAT Node：采集贴片1
			controller->slavePool().add<aris::control::EthercatSlave>();
            controller->slavePool().back().setPhyId(5);
			dynamic_cast<aris::control::EthercatSlave&>(controller->slavePool().back()).scanInfoForCurrentSlave();
			dynamic_cast<aris::control::EthercatSlave&>(controller->slavePool().back()).scanPdoForCurrentSlave();
			//添加第7个EtherCAT Node：采集贴片2
			controller->slavePool().add<aris::control::EthercatSlave>();
            controller->slavePool().back().setPhyId(6);
			dynamic_cast<aris::control::EthercatSlave&>(controller->slavePool().back()).scanInfoForCurrentSlave();
			dynamic_cast<aris::control::EthercatSlave&>(controller->slavePool().back()).scanPdoForCurrentSlave();
			//添加第8个EtherCAT Node：采集贴片3
			controller->slavePool().add<aris::control::EthercatSlave>();
            controller->slavePool().back().setPhyId(7);
			dynamic_cast<aris::control::EthercatSlave&>(controller->slavePool().back()).scanInfoForCurrentSlave();
			dynamic_cast<aris::control::EthercatSlave&>(controller->slavePool().back()).scanPdoForCurrentSlave();
			//添加第9个EtherCAT Node：采集贴片4
			controller->slavePool().add<aris::control::EthercatSlave>();
            controller->slavePool().back().setPhyId(8);
			dynamic_cast<aris::control::EthercatSlave&>(controller->slavePool().back()).scanInfoForCurrentSlave();
			dynamic_cast<aris::control::EthercatSlave&>(controller->slavePool().back()).scanPdoForCurrentSlave();

#ifndef WIN32
            //dynamic_cast<aris::control::EthercatMotion&>(controller->slavePool().back()).scanInfoForCurrentSlave();
#endif

		return controller;
};
    //set DH parameters
    auto createModelRokae()->std::unique_ptr<aris::dynamic::Model>
	{
		aris::dynamic::PumaParam param;
        param.d1 = 0.3295;
        param.a1 = 0.04;
        param.a2 = 0.275;
		param.d3 = 0.0;
        param.a3 = 0.025;
        param.d4 = 0.28;

        param.tool0_pe[2] = 0.078;

        auto model = aris::dynamic::createModelPuma(param);
		/*
		//根据tool0，添加一个tool1，tool1相对于tool0在x方向加上0.1m//
		auto &tool0 = model->partPool().back().markerPool().findByName("general_motion_0_i");//获取tool0

		double pq_ee_i[7];
		s_pm2pq(*tool0->prtPm(), pq_ee_i);
		pq_ee_i[0] += 0.1;//在tool0的x方向加上0.1m

		double pm_ee_i[16];
		s_pq2pm(pq_ee_i, pm_ee_i);

		auto &tool1 = model->partPool().back().markerPool().add<Marker>("tool1", pm_ee_i);//添加tool1

		//在根据tool1位姿反解到每一个关节时，需要调用下面两行代码来实现
		//tool1.setPm(pm_ee_i);
		//model->generalMotionPool()[0].updMpm();
		*/
		return std::move(model);
	}

	// 单关节正弦往复轨迹 //
	struct moveJ_CosParam
	{
		double j[6];
		double time;
		uint32_t timenum;
		std::vector<bool> joint_active_vec;
	};
	auto moveJ_Cos::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
		//moveJ_CosParam param = {{0.0,0.0,0.0,0.0,0.0,0.0},0.0,0};
		moveJ_CosParam param;
		for (Size i = 0; i < 6; i++)
		{
			param.j[i] = 0.0;
		}
		param.time = 0.0;
		param.timenum = 0;

		param.joint_active_vec.clear();
		param.joint_active_vec.resize(target.model->motionPool().size(), true);

		for (auto &p : params)//       范围for 遍历 
		{
			if (p.first == "j1")
			{
				if (p.second == "current_pos")
				{
					param.joint_active_vec[0] = false;
					param.j[0] = target.model->motionPool()[0].mp();
				}
				else
				{
					param.joint_active_vec[0] = true;
					param.j[0] = std::stod(p.second);
				}

			}
			else if (p.first == "j2")
			{
				if (p.second == "current_pos")
				{
					param.joint_active_vec[1] = false;
					param.j[1] = target.model->motionPool()[1].mp();
				}
				else
				{
					param.joint_active_vec[1] = true;
					param.j[1] = std::stod(p.second);
				}
			}
			else if (p.first == "j3")
			{
				if (p.second == "current_pos")
				{
					param.joint_active_vec[2] = false;
					param.j[2] = target.model->motionPool()[2].mp();
				}
				else
				{
					param.joint_active_vec[2] = true;
					param.j[2] = std::stod(p.second);
				}
			}
			else if (p.first == "j4")
			{
				if (p.second == "current_pos")
				{
					param.joint_active_vec[3] = false;
					param.j[3] = target.model->motionPool()[3].mp();
				}
				else
				{
					param.joint_active_vec[3] = true;
					param.j[3] = std::stod(p.second);
				}
			}
			else if (p.first == "j5")
			{
				if (p.second == "current_pos")
				{
					param.joint_active_vec[4] = false;
					param.j[4] = target.model->motionPool()[4].mp();
				}
				else
				{
					param.joint_active_vec[4] = true;
					param.j[4] = std::stod(p.second);
				}
			}
			else if (p.first == "j6")
			{
				if (p.second == "current_pos")
				{
					param.joint_active_vec[5] = false;
					param.j[5] = target.model->motionPool()[5].mp();
				}
				else
				{
					param.joint_active_vec[5] = true;
					param.j[5] = std::stod(p.second);
				}
			}
			else if (p.first == "time")
			{
				param.time = std::stod(p.second);
			}
			else if (p.first == "timenum")
			{
				param.timenum = std::stoi(p.second);
			}
		}
		target.param = param;

		target.option |=
			Plan::USE_TARGET_POS |
#ifdef WIN32
			Plan::NOT_CHECK_POS_MIN |
			Plan::NOT_CHECK_POS_MAX |
			Plan::NOT_CHECK_POS_CONTINUOUS |
			Plan::NOT_CHECK_POS_CONTINUOUS_AT_START |
			Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER |
			Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER_AT_START |
			Plan::NOT_CHECK_POS_FOLLOWING_ERROR |
#endif
			Plan::NOT_CHECK_VEL_MIN |
			Plan::NOT_CHECK_VEL_MAX |
			Plan::NOT_CHECK_VEL_CONTINUOUS |
            Plan::NOT_CHECK_VEL_CONTINUOUS_AT_START |
			Plan::NOT_CHECK_VEL_FOLLOWING_ERROR;

	}
	auto moveJ_Cos::executeRT(PlanTarget &target)->int
	{
		auto &param = std::any_cast<moveJ_CosParam&>(target.param);
		auto time = static_cast<int32_t>(param.time * 1000);
		auto totaltime = static_cast<int32_t>(param.timenum * time);
		static double begin_pjs[6];
		static double step_pjs[6];

		// 获取当前起始点位置 //
		if (target.count == 1)
		{
			for (Size i = 0; i < param.joint_active_vec.size(); ++i)
			{
				begin_pjs[i] = target.model->motionPool()[i].mp();
				step_pjs[i] = target.model->motionPool()[i].mp();
			}
		}
		if ((int)floor(2.0*target.count / time) % 2 == 0)                                        //??????
		{
			for (Size i = 0; i < 1; ++i)//param.joint_active_vec.size() to 1
			{
				step_pjs[i] = begin_pjs[i] + param.j[i] * (1 - std::cos(4 * PI*target.count / time)) / 2;
				target.model->motionPool().at(i).setMp(step_pjs[i]);
			}
		}
		else
		{
			for (Size i = 0; i < param.joint_active_vec.size(); ++i)
			{
				step_pjs[i] = begin_pjs[i] - param.j[i] * (1 - std::cos(4 * PI*target.count / time)) / 2;
				target.model->motionPool().at(i).setMp(step_pjs[i]);
			}
		}

		//at(i)    意思是【i】
		if (target.model->solverPool().at(1).kinPos())return -1;

		// 访问主站 //
		auto controller = target.controller;

		// 打印电流 //
		auto &cout = controller->mout();
		if (target.count % 100 == 0)\


		{
			for (Size i = 0; i < 6; i++)
			{
				cout << "pos" << i + 1 << ":" << controller->motionAtAbs(i).actualPos() << "  ";
				cout << "vel" << i + 1 << ":" << controller->motionAtAbs(i).actualVel() << "  ";
				cout << "cur" << i + 1 << ":" << controller->motionAtAbs(i).actualCur() << "  ";
			}
			cout << std::endl;
		}

		// log 电流 //
		auto &lout = controller->lout();
		for (Size i = 0; i < 6; i++)
		{
			lout << controller->motionAtAbs(i).targetPos() << ",";
			lout << controller->motionAtAbs(i).actualPos() << ",";
			lout << controller->motionAtAbs(i).actualVel() << ",";
			lout << controller->motionAtAbs(i).actualCur() << ",";
		}
		lout << std::endl;

		return totaltime - target.count;
	}
	auto moveJ_Cos::collectNrt(PlanTarget &target)->void {}
	moveJ_Cos::moveJ_Cos(const std::string &name) :Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"moveJ_Cos\">"
			"	<GroupParam>"
			"		<Param name=\"j1\" default=\"current_pos\"/>"
			"		<Param name=\"j2\" default=\"current_pos\"/>"
			"		<Param name=\"j3\" default=\"current_pos\"/>"
			"		<Param name=\"j4\" default=\"current_pos\"/>"
			"		<Param name=\"j5\" default=\"current_pos\"/>"
			"		<Param name=\"j6\" default=\"current_pos\"/>"
			"		<Param name=\"time\" default=\"1.0\" abbreviation=\"t\"/>"
			"		<Param name=\"timenum\" default=\"2\" abbreviation=\"n\"/>"
			"	</GroupParam>"
			"</Command>");
	}

	// 单关节相对运动轨迹--输入单个关节，角度位置；关节按照梯形速度轨迹执行；速度前馈//
	struct moveJ_TParam
	{
		double vel, acc, dec;
		std::vector<double> joint_pos_vec, begin_joint_pos_vec;
		std::vector<bool> joint_active_vec;
	};
	auto moveJ_T::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
		auto c = target.controller;
		moveJ_TParam param;

		for (auto cmd_param : params)
		{
			if (cmd_param.first == "all")
			{
				param.joint_active_vec.resize(target.model->motionPool().size(), true);
			}
			else if (cmd_param.first == "none")
			{
				param.joint_active_vec.resize(target.model->motionPool().size(), false);
			}
			else if (cmd_param.first == "motion_id")
			{
				param.joint_active_vec.resize(target.model->motionPool().size(), false);
				param.joint_active_vec.at(std::stoi(cmd_param.second)) = true;
			}
			else if (cmd_param.first == "physical_id")
			{
				param.joint_active_vec.resize(c->motionPool().size(), false);
				param.joint_active_vec.at(c->motionAtPhy(std::stoi(cmd_param.second)).phyId()) = true;
			}
			else if (cmd_param.first == "slave_id")
			{
				param.joint_active_vec.resize(c->motionPool().size(), false);
				param.joint_active_vec.at(c->motionAtPhy(std::stoi(cmd_param.second)).slaId()) = true;
			}
			else if (cmd_param.first == "pos")
			{
				aris::core::Matrix mat = target.model->calculator().calculateExpression(cmd_param.second);
				if (mat.size() == 1)param.joint_pos_vec.resize(c->motionPool().size(), mat.toDouble());
				else
				{
					param.joint_pos_vec.resize(mat.size());
					std::copy(mat.begin(), mat.end(), param.joint_pos_vec.begin());
				}
			}
			else if (cmd_param.first == "vel")
			{
				param.vel = std::stod(cmd_param.second);
			}
			else if (cmd_param.first == "acc")
			{
				param.acc = std::stod(cmd_param.second);
			}
			else if (cmd_param.first == "dec")
			{
				param.dec = std::stod(cmd_param.second);
			}
		}

		param.begin_joint_pos_vec.resize(target.model->motionPool().size());

		target.param = param;

		target.option |=
			//				Plan::USE_TARGET_POS |
            Plan::USE_VEL_OFFSET |
#ifdef WIN32
			Plan::NOT_CHECK_POS_MIN |
			Plan::NOT_CHECK_POS_MAX |
			Plan::NOT_CHECK_POS_CONTINUOUS |
			Plan::NOT_CHECK_POS_CONTINUOUS_AT_START |
			Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER |
			Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER_AT_START |
			Plan::NOT_CHECK_POS_FOLLOWING_ERROR |
#endif
			Plan::NOT_CHECK_VEL_MIN |
			Plan::NOT_CHECK_VEL_MAX |
			Plan::NOT_CHECK_VEL_CONTINUOUS |
            Plan::NOT_CHECK_VEL_CONTINUOUS_AT_START |
			Plan::NOT_CHECK_VEL_FOLLOWING_ERROR;

	}
	auto moveJ_T::executeRT(PlanTarget &target)->int
	{
		auto &param = std::any_cast<moveJ_TParam&>(target.param);
		auto controller = target.controller;

		if (target.count == 1)
		{
			for (Size i = 0; i < param.joint_active_vec.size(); ++i)//对所有电机
			{
				if (param.joint_active_vec[i])//使能了
				{
					param.begin_joint_pos_vec[i] = controller->motionAtAbs(i).actualPos();//获取位置
				}
			}
		}

		aris::Size total_count{ 1 };
		for (Size i = 0; i < param.joint_active_vec.size(); ++i)
		{
			if (param.joint_active_vec[i])
			{
				double p, v, a;
				aris::Size t_count;
				aris::plan::moveAbsolute(target.count, param.begin_joint_pos_vec[i], param.begin_joint_pos_vec[i] + param.joint_pos_vec[i], 
					param.vel / 1000, param.acc / 1000 / 1000, param.dec / 1000 / 1000, p, v, a, t_count);//当前位置+指令给的距离
				controller->motionAtAbs(i).setTargetPos(p);
				controller->motionAtAbs(i).setTargetVel(v * 1000);
				total_count = std::max(total_count, t_count);

				target.model->motionPool().at(i).setMp(p);
			}
		}

		//controller与模型同步，保证3D仿真模型同步显示
		if (target.model->solverPool().at(1).kinPos())return -1; // ????

		// 打印电流 //
		auto &cout = controller->mout();
		if (target.count % 100 == 0)
		{
			for (Size i = 0; i < 6; i++)
			{
				cout << "mp" << i + 1 << ":" << target.model->motionPool()[i].mp() << "  ";
				cout << "pos" << i + 1 << ":" << controller->motionAtAbs(i).targetPos() << "  ";
				cout << "vel" << i + 1 << ":" << controller->motionAtAbs(i).actualVel() << "  ";
				cout << "cur" << i + 1 << ":" << controller->motionAtAbs(i).actualCur() << "  ";
			}
			cout << std::endl;
		}

		// log 电流 //
		auto &lout = controller->lout();
		for (Size i = 0; i < 6; i++)
		{
			lout << controller->motionAtAbs(i).targetPos() << ",";
			lout << controller->motionAtAbs(i).actualPos() << ",";
			lout << controller->motionAtAbs(i).actualVel() << ",";
			lout << controller->motionAtAbs(i).actualCur() << ",";
		}
		lout << std::endl;

		return total_count - target.count;//返回剩余     count=t_count  自动加1？
	}
	auto moveJ_T::collectNrt(PlanTarget &target)->void {}
	moveJ_T::moveJ_T(const std::string &name) :Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"moveJ_T\">"
			"	<GroupParam>"
			"		<UniqueParam default=\"all\">"
			"			<Param name=\"all\" abbreviation=\"a\"/>"
			"			<Param name=\"motion_id\" abbreviation=\"m\" default=\"0\"/>"
			"			<Param name=\"physical_id\" abbreviation=\"p\" default=\"0\"/>"
			"			<Param name=\"slave_id\" abbreviation=\"s\" default=\"0\"/>"
			"		</UniqueParam>"
			"		<Param name=\"pos\" default=\"0\"/>"
			"		<Param name=\"vel\" default=\"0.5\"/>"
			"		<Param name=\"acc\" default=\"1\"/>"
			"		<Param name=\"dec\" default=\"1\"/>"
			"		<UniqueParam default=\"check_none\">"
			"			<Param name=\"check_all\"/>"
			"			<Param name=\"check_none\"/>"
			"			<GroupParam>"
			"				<UniqueParam default=\"check_pos\">"
			"					<Param name=\"check_pos\"/>"
			"					<Param name=\"not_check_pos\"/>"
			"					<GroupParam>"
			"						<UniqueParam default=\"check_pos_max\">"
			"							<Param name=\"check_pos_max\"/>"
			"							<Param name=\"not_check_pos_max\"/>"
			"						</UniqueParam>"
			"						<UniqueParam default=\"check_pos_min\">"
			"							<Param name=\"check_pos_min\"/>"
			"							<Param name=\"not_check_pos_min\"/>"
			"						</UniqueParam>"
			"						<UniqueParam default=\"check_pos_continuous\">"
			"							<Param name=\"check_pos_continuous\"/>"
			"							<Param name=\"not_check_pos_continuous\"/>"
			"						</UniqueParam>"
			"						<UniqueParam default=\"check_pos_continuous_at_start\">"
			"							<Param name=\"check_pos_continuous_at_start\"/>"
			"							<Param name=\"not_check_pos_continuous_at_start\"/>"
			"						</UniqueParam>"
			"						<UniqueParam default=\"check_pos_continuous_second_order\">"
			"							<Param name=\"check_pos_continuous_second_order\"/>"
			"							<Param name=\"not_check_pos_continuous_second_order\"/>"
			"						</UniqueParam>"
			"						<UniqueParam default=\"check_pos_continuous_second_order_at_start\">"
			"							<Param name=\"check_pos_continuous_second_order_at_start\"/>"
			"							<Param name=\"not_check_pos_continuous_second_order_at_start\"/>"
			"						</UniqueParam>"
			"						<UniqueParam default=\"check_pos_following_error\">"
			"							<Param name=\"check_pos_following_error\"/>"
			"							<Param name=\"not_check_pos_following_error\"/>"
			"						</UniqueParam>"
			"					</GroupParam>"
			"				</UniqueParam>"
			"				<UniqueParam default=\"check_vel\">"
			"					<Param name=\"check_vel\"/>"
			"					<Param name=\"not_check_vel\"/>"
			"					<GroupParam>"
			"						<UniqueParam default=\"check_vel_max\">"
			"							<Param name=\"check_vel_max\"/>"
			"							<Param name=\"not_check_vel_max\"/>"
			"						</UniqueParam>"
			"						<UniqueParam default=\"check_vel_min\">"
			"							<Param name=\"check_vel_min\"/>"
			"							<Param name=\"not_check_vel_min\"/>"
			"						</UniqueParam>"
			"						<UniqueParam default=\"check_vel_continuous\">"
			"							<Param name=\"check_vel_continuous\"/>"
			"							<Param name=\"not_check_vel_continuous\"/>"
			"						</UniqueParam>"
			"						<UniqueParam default=\"check_vel_continuous_at_start\">"
			"							<Param name=\"check_vel_continuous_at_start\"/>"
			"							<Param name=\"not_check_vel_continuous_at_start\"/>"
			"						</UniqueParam>"
			"						<UniqueParam default=\"check_vel_following_error\">"
			"							<Param name=\"check_vel_following_error\"/>"
			"							<Param name=\"not_check_vel_following_error\"/>"
			"						</UniqueParam>"
			"					</GroupParam>"
			"				</UniqueParam>"
			"			</GroupParam>"
			"		</UniqueParam>"
			"	</GroupParam>"
			"</Command>");
	}

	struct ShowAllParam
	{
		std::vector<double> axis_pos_vec;
		std::vector<double> axis_pq_vec;
	};
	auto ShowAll::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
		ShowAllParam param;
		param.axis_pos_vec.clear();
		param.axis_pq_vec.clear();
        param.axis_pos_vec.resize(3, 0.0);
        param.axis_pq_vec.resize(4, 0.0);
		target.param = param;

		std::fill(target.mot_options.begin(), target.mot_options.end(),
			Plan::USE_TARGET_POS);
	}
	auto ShowAll::executeRT(PlanTarget &target)->int
	{
		// 访问主站 //
		auto controller = target.controller;
		auto &param = std::any_cast<ShowAllParam&>(target.param);

		// 取得起始位置 //
		for (Size i = 0; i < param.axis_pos_vec.size(); ++i)
		{
			param.axis_pos_vec[i] = controller->motionAtAbs(i).actualPos();//电机当前角度
		}

        target.model->generalMotionPool().at(0).getMpq(param.axis_pq_vec.data());

		// 打印 //
		auto &cout = controller->mout();
		cout << "current pq:" << std::endl;
        for (Size i = 0; i < 3; i++)
		{
			cout << param.axis_pq_vec[i] << " ";
		}
		cout << std::endl;
		cout << "current pos:" << std::endl;
        for (Size i = 0; i < 3; i++)
		{
			cout << param.axis_pos_vec[i] << "  ";
		}
		cout << std::endl;

		// log //
		auto &lout = controller->lout();
        for (Size i = 0; i < 3; i++)
		{
			lout << param.axis_pos_vec[i] << " ";
		}
		lout << std::endl;
		return 0;
	}
	auto ShowAll::collectNrt(PlanTarget &target)->void {}
	ShowAll::ShowAll(const std::string &name) : Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"sha\">"
			"</Command>");
	}

	struct MoveJRParam
	{
		double begin_pos, target_pos, vel, acc, dec;
		std::vector<bool> joint_active_vec;
	};
	auto MoveJR::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
		MoveJRParam param;

        param.joint_active_vec.clear();

		for (auto cmd_param : params)
		{
			if (cmd_param.first == "motion_id")
			{
                param.joint_active_vec.resize(target.model->motionPool().size(), false);
                param.joint_active_vec.at(std::stoi(cmd_param.second)) = true;
			}
			else if (cmd_param.first == "pos")
			{
				param.target_pos = std::stod(cmd_param.second);
			}
			else if (cmd_param.first == "vel")
			{
				param.vel = std::stod(cmd_param.second);
			}
			else if (cmd_param.first == "acc")
			{
				param.acc = std::stod(cmd_param.second);
			}
			else if (cmd_param.first == "dec")
			{
				param.dec = std::stod(cmd_param.second);
			}
		}

		target.param = param;

        //std::fill(target.mot_options.begin(), target.mot_options.end(), Plan::USE_TARGET_POS);
	}
	auto MoveJR::executeRT(PlanTarget &target)->int
	{
		auto &param = std::any_cast<MoveJRParam&>(target.param);
		auto controller = target.controller;
		

		//获取第一个count时，电机的当前角度位置//
		if (target.count == 1)
		{
            for (Size i = 0; i < 3; ++i)//param.joint_active_vec.size() to 1
			{
				if (param.joint_active_vec[i])
				{
                    param.begin_pos = controller->motionAtAbs(i).actualPos();
                    //setting elmo driver max. torque
                    auto &ec_mot = static_cast<aris::control::EthercatMotion&>(controller->motionAtPhy(i));
                    ec_mot.writePdo(0x6072, 0x00, std::int16_t(1000));
				}
			}


		}

		//梯形轨迹//
		aris::Size total_count{ 1 };
        for (Size i = 0; i < 3; ++i)//param.joint_active_vec.size() to 1
		{
			if (param.joint_active_vec[i])
			{
				double p, v, a;
				aris::Size t_count;
				aris::plan::moveAbsolute(target.count, param.begin_pos, param.begin_pos + param.target_pos, 
					param.vel / 1000, param.acc / 1000 / 1000, param.dec / 1000 / 1000, p, v, a, t_count); //规划位置  修改P。   自动count+1?
				controller->motionAtAbs(i).setTargetPos(p);  //和setMP 有点区别   //电机就动了
                //target.model->motionPool().at(i).setMp(p);  //模型
				total_count = std::max(total_count, t_count);
			}
		}
		//3D模型同步
        if (target.model->solverPool().at(1).kinPos())return -1;


		// 打印 //
		auto &cout = controller->mout();
		if (target.count % 100 == 0)
		{
            //cout << "target_pos" << ":" << param.target_pos << " ";
            //cout << "vel" << ":" << param.vel << " ";
            //cout << "acc" << ":" << param.acc << " ";
            //cout << "dec"  << ":" << param.dec << " ";
			
            for (Size i = 0; i < 3; ++i)//param.joint_active_vec.size() to 1
			{
				if (param.joint_active_vec[i])
				{
                    cout << "targetPos" << ":" << controller->motionAtPhy(i).targetPos()<< " ";
                    //cout << "actualVel" << ":" << controller->motionAtAbs(i).actualVel() << " ";
                    cout << "actualPos" << ":" << controller->motionAtPhy(i).actualPos() << " ";
				}
			}

			cout << std::endl;
		}

		// log //
		auto &lout = controller->lout();
        for (Size i = 0; i <3; i++)//param.joint_active_vec.size() to 1
		{
			lout << controller->motionAtAbs(i).targetPos() << ",";
			lout << controller->motionAtAbs(i).actualPos() << ",";
         //   lout << controller->motionAtAbs(i).actualVel() << ",";
            lout << controller->motionAtAbs(i).actualTor() << ",";
		}
		lout << std::endl;

		return total_count - target.count;
	}
	auto MoveJR::collectNrt(PlanTarget &target)->void {}
	MoveJR::MoveJR(const std::string &name) :Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"moveJR\">"
			"	<GroupParam>"
			"		<Param name=\"motion_id\" abbreviation=\"m\" default=\"0\"/>"
			"		<Param name=\"pos\" default=\"0\"/>"
			"		<Param name=\"vel\" abbreviation=\"v\" default=\"0.5\"/>"
			"		<Param name=\"acc\" default=\"1\"/>"
			"		<Param name=\"dec\" default=\"1\"/>"
			"	</GroupParam>"
			"</Command>");
	}

    struct SensorParam
    {
        int time;

        std::vector<std::vector<double>> channel;
        std::vector<std::vector<std::vector<double>>> acc,vel,ang;//第i个采集卡，第j个通道，每个通道3个方向
        std::vector<double> a1, a2, w1, w2, Angle1, Angle2;//类内不要初始化

    };
    auto Sensor::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
    {
        SensorParam param;
        param.channel.clear();//记得清除
        std::vector<double> init= { 0,0,0 ,0};
        //初始化channel,完成后每个板卡都是init
        for (Size i = 5; i <=8; i++)
        {
            param.channel.push_back(init);
        }

        param.acc.clear();
        param.vel.clear();
        param.ang.clear();
        param.a1.clear();
        param.a2.clear();
        param.w1.clear();
        param.w2.clear();
        param.Angle1.clear();
        param.Angle2.clear();


        std::vector<std::vector<double>> init_imu(2, { 0,0,0 });
        param.a1 = { 0, 0, 0 };
        param.a2 = { 0, 0, 0 };
        param.w1 = { 0, 0, 0 };
        param.w1 = { 0, 0, 0 };
        param.Angle1 = { 0, 0, 0 };
        param.Angle2 = { 0, 0, 0 };


        for (auto &p : params) //遍历for   加了&使得独到的值可修改
        {
            if (p.first == "time")
            {
                param.time = std::stoi(p.second);
            }

        }

        target.param = param;
        //将所有option设置为NOT_CHECK_ENABLE
        std::fill(target.mot_options.begin(), target.mot_options.end(), Plan::NOT_CHECK_ENABLE);
    }
    auto Sensor::executeRT(PlanTarget &target)->int
    {
        auto &param = std::any_cast<SensorParam&>(target.param);
        // 访问主站 //
        auto controller = dynamic_cast<aris::control::EthercatController*>(target.controller);

        int16_t rawData;
        double volToSig = 10.0;
        //电机下标为at(0,1,2),耦合器下标at(3,4),
        //读取PDO，第一参数为index，第二参数为subindex，第三参数读取数据，第四参数为操作位数
        //16位精度
        for (Size i = 5; i <=8; ++i)//
        {
            controller->slavePool().at(i).readPdo(0x6000, 0x11, &rawData, 16);
            param.channel[i-5][0] = volToSig * rawData / 32767;//2^15-1=32767
            controller->slavePool().at(i).readPdo(0x6010, 0x11, &rawData, 16);
            param.channel[i-5][1] = volToSig * rawData / 32767;
            controller->slavePool().at(i).readPdo(0x6020, 0x11, &rawData, 16);
            param.channel[i-5][2] = volToSig * rawData / 32767;
            controller->slavePool().at(i).readPdo(0x6030, 0x11, &rawData, 16);
            param.channel[i-5][3] = volToSig * rawData / 32767;
        }
        std::vector<char> imudata1(22,0), imudata2(22,0);//初始化，共22个数，均为0
        std::vector<double> a1(3,0), a2(3,0), w1(2,0), w2(3,0), Angle1(3,0), Angle2(3,0);
        char rawimuData;//EL6xxx找到reversionNo=0x00130000的；显示每个data out 的bitsize为8；但type却是USINT，即unsigned short int，16位？



        //print//
        //setw(n)设置输出宽度为n
        auto &cout = controller->mout();
        if (target.count % 100 == 0)
        {
           /* for (Size i = 5; i <=8; ++i)
            {

                cout << std::setw(6) << param.channel[i-5][0] << "  ";//std::setw(6)  对齐功能？设置宽度
                cout << std::setw(6) << param.channel[i-5][1] << "  ";
                cout << std::setw(6) << param.channel[i-5][2] << "  ";
                cout << std::setw(6) << param.channel[i-5][3] << "  ";

            }
            */
            for (Size i = 5; i <=5; ++i)
            {

                //cout << std::setw(6) << param.channel[i-5][0] << "  ";//std::setw(6)  对齐功能？设置宽度
                //cout << std::setw(6) << param.channel[i-5][1] << "  ";
                cout << std::setw(6) << param.channel[i-5][2] << "  ";
                //cout << std::setw(6) << param.channel[i-5][3] << "  ";

            }
            for (Size i = 6; i <=6; ++i)
            {

                cout << std::setw(6) << param.channel[i-5][0] << "  ";//std::setw(6)  对齐功能？设置宽度
                cout << std::setw(6) << param.channel[i-5][1] << "  ";
                //cout << std::setw(6) << param.channel[i-5][2] << "  ";
                //cout << std::setw(6) << param.channel[i-5][3] << "  ";

            }
            cout << std::endl;
            cout << "----------------------------------------------------" << std::endl;

        }


        //log//
        auto &lout = controller->lout();
        /*for (Size i = 5; i <=8; ++i)
        {
            lout << param.channel[i-5][0] << " ";
            lout << param.channel[i-5][1] << " ";
            lout << param.channel[i-5][2] << " ";
            lout << param.channel[i-5][3] << " ";
        }*/
        for (Size i = 5; i <=5; ++i)
        {
            //lout << param.channel[i-5][0] << " ";
            //lout << param.channel[i-5][1] << " ";
            lout << param.channel[i-5][2] << " ";
            //lout << param.channel[i-5][3] << " ";
        }
        for (Size i = 6; i <=6; ++i)
        {
            lout << param.channel[i-5][0] << " ";
            lout << param.channel[i-5][1] << " ";
            //lout << param.channel[i-5][2] << " ";
            //lout << param.channel[i-5][3] << " ";
        }

        lout << std::endl;
        //时间为0则结束循环
        param.time--;
        return param.time;    //return的为0就停止循环？
    }
    auto Sensor::collectNrt(PlanTarget &target)->void {}
    Sensor::Sensor(const std::string &name) :Plan(name)
    {
        command().loadXmlStr(
            "<Command name=\"sensor\">"
            "	<GroupParam>"
            "		<Param name=\"time\" default=\"30000\"/>"
            "	</GroupParam>"
            "</Command>");
    }

    struct MoveJDParam
        {
            int Motion_dir;// { ShouAbd,ShouAdd,ShouFle,ShouEx,ElbFle,ElbEx	}
            int time;
            double begin_pos, target_pos, vel, acc, dec;
            std::vector<bool> joint_active_vec;
        };
    auto MoveJD::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
    {
        MoveJDParam param;

        param.joint_active_vec.clear();

        for (auto cmd_param : params)
        {
            if (cmd_param.first == "vel")
            {
                param.vel = std::stod(cmd_param.second);
            }
            else if (cmd_param.first == "acc")
            {
                param.acc = std::stod(cmd_param.second);
            }
            else if (cmd_param.first == "dec")
            {
                param.dec = std::stod(cmd_param.second);
            }
            else if (cmd_param.first == "direction")
            {
                param.Motion_dir = std::stoi(cmd_param.second);
                param.joint_active_vec.resize(target.model->motionPool().size(), false);

                switch (param.Motion_dir)
                {
                case 1:
                    param.joint_active_vec.at(0) = true;
                    break;
                case 2:
                    param.joint_active_vec.at(0) = true;
                    break;
                case 3:
                    param.joint_active_vec.at(1) = true;
                    break;
                case 4:
                    param.joint_active_vec.at(1) = true;
                    break;
                case 5:
                    param.joint_active_vec.at(2) = true;
                    break;
                case 6:
                    param.joint_active_vec.at(2) = true;
                    break;
                default:
                    break;
                }

            }
            else if (cmd_param.first == "time")
            {
                param.time = std::stoi(cmd_param.second);
            }
        }




        target.param = param;

        target.option |=
            Plan::USE_TARGET_POS |
#ifdef WIN32
            Plan::NOT_CHECK_POS_CONTINUOUS |
            Plan::NOT_CHECK_POS_CONTINUOUS_AT_START |
            Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER |
            Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER_AT_START |
            Plan::NOT_CHECK_POS_FOLLOWING_ERROR |
#endif
            Plan::NOT_CHECK_VEL_CONTINUOUS |
            Plan::NOT_CHECK_VEL_CONTINUOUS_AT_START |
            Plan::NOT_CHECK_POS_CONTINUOUS_AT_START |
            Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER |
            Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER_AT_START |
            Plan::NOT_CHECK_VEL_FOLLOWING_ERROR;

        //std::fill(target.mot_options.begin(), target.mot_options.end(), Plan::USE_TARGET_POS);
        //我自己加的
        //std::fill(target.mot_options.begin(), target.mot_options.end(), Plan::NOT_CHECK_ENABLE);

    }
    auto MoveJD::executeRT(PlanTarget &target)->int
    {
        auto &param = std::any_cast<MoveJDParam&>(target.param);
        auto controller = target.controller;


        //获取第一个count时，电机的当前角度位置//
        if (target.count == 1)
        {
            for (Size i = 0; i < 3; ++i)//param.joint_active_vec.size() to 1
            {
                if (param.joint_active_vec[i])
                {
                    param.begin_pos = controller->motionAtAbs(i).actualPos();
                }
            }
            //setting elmo driver max. torque
            auto &ec_mot = static_cast<aris::control::EthercatMotion&>(controller->motionAtPhy(0));
            ec_mot.writePdo(0x6072, 0x00, std::int16_t(1000));
            auto &ec_mot1 = static_cast<aris::control::EthercatMotion&>(controller->motionAtPhy(1));
            ec_mot1.writePdo(0x6072, 0x00, std::int16_t(1000));
            auto &ec_mot2 = static_cast<aris::control::EthercatMotion&>(controller->motionAtPhy(2));
            ec_mot2.writePdo(0x6072, 0x00, std::int16_t(1000));
        }

        //轨迹//
        aris::Size total_count{ 1 };
        for (Size i = 0; i < 3; ++i)//param.joint_active_vec.size() to 1
        {
            if (param.joint_active_vec[i])
            {
                double p, v, a;
                aris::Size t_count;

                //轨迹规划
                switch (param.Motion_dir)
                {


                case 1:
                case 3:
                case 5:
                    param.target_pos = 10.0 / 180.0 * PI;//总运动距离
                    //p = param.begin_pos + 1.0 * target.count * param.target_pos / param.time;
                    aris::plan::moveAbsolute(target.count, param.begin_pos, param.begin_pos + param.target_pos,
                        param.vel / 1000, param.acc / 100 / 1000, param.dec / 100 / 1000, p, v, a, t_count); //规划位置  修改P。   自动count+1?



                    break;
                case 2:
                case 4:
                case 6:
                    param.target_pos = 10.0 / 180.0 * PI;
                    aris::plan::moveAbsolute(target.count, param.begin_pos, param.begin_pos - param.target_pos,
                        param.vel / 1000, param.acc / 100 / 1000, param.dec / 100 / 1000, p, v, a, t_count); //规划位置  修改P。   自动target.count+1?

                    //p = param.begin_pos - 1.0 * target.count * param.target_pos / 500;// / param.time;
                    break;
                }
                total_count = std::max(total_count, t_count);

                controller->motionAtAbs(i).setTargetPos(p);  //和setMP 有点区别   //电机就动了
                target.model->motionPool().at(i).setMp(p);  //模型
                //total_count = std::max(total_count, t_count);
            }
        }
        //3D模型同步
        //if (target.model->solverPool().at(i).kinPos())return -1;


        // 打印 //
        auto &cout = controller->mout();
        if (target.count % 100 == 0)
        {
            //cout << "target_pos" << ":" << param.target_pos << " ";
            //cout << "vel" << ":" << param.vel << " ";
            //cout << "acc" << ":" << param.acc << " ";
            //cout << "dec"  << ":" << param.dec << " ";

            for (Size i = 0; i < 3; ++i)//param.joint_active_vec.size() to 1
            {
                if (param.joint_active_vec[i])
                {
                    cout << "targetPos" << ":" << controller->motionAtPhy(i).targetPos() << " ";
                    //cout << "actualVel" << ":" << controller->motionAtAbs(i).actualVel() << " ";
                    cout << "actualPos" << ":" << controller->motionAtPhy(i).actualPos() << " ";
                }
            }

            cout << std::endl;
        }

        // log //
        /*auto &lout = controller->lout();
        for (Size i = 0; i < 1; i++)//param.joint_active_vec.size() to 1
        {
            lout << controller->motionAtAbs(i).targetPos() << ",";
            lout << controller->motionAtAbs(i).actualPos() << ",";
            //   lout << controller->motionAtAbs(i).actualVel() << ",";
            lout << controller->motionAtAbs(i).actualTor() << ",";
        }
        lout << std::endl;
        */
        return param.time - target.count;
    }
    auto MoveJD::collectNrt(PlanTarget &target)->void {}
    MoveJD::MoveJD(const std::string &name) :Plan(name)
    {
        command().loadXmlStr(
            "<Command name=\"moveJD\">"
            "	<GroupParam>"
            "		<Param name=\"direction\" abbreviation=\"d\" default=\"0\"/>"
            "		<Param name=\"time\" abbreviation=\"t\" default=\"500\"/>"
            "		<Param name=\"pos\" default=\"0\"/>"
            "		<Param name=\"vel\" abbreviation=\"v\" default=\"0.5\"/>"
            "		<Param name=\"acc\" default=\"1\"/>"
            "		<Param name=\"dec\" default=\"1\"/>"
            "	</GroupParam>"
            "</Command>");
    }

    struct MoveCosFCParam
    {
        double begin_pos, target_pos, amp, cycle, phi0, offset;
        double amp_inc=0, offset_inc=0;
        int total_time;//持续时间，默认5000count
        std::vector<std::vector<double>> channel;
        std::vector<bool> joint_active_vec;
    };
    auto MoveCosFC::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
    {
        auto c = target.controller;
        MoveCosFCParam param;

        for (auto cmd_param : params)
        {
            if (cmd_param.first == "motion_id")
            {
                param.joint_active_vec.resize(target.model->motionPool().size(), false);
                //param.joint_active_vec.at(std::stoi(cmd_param.second)) = true;
                param.joint_active_vec.at(2) = true;//只是第三轴
            }
            else if (cmd_param.first == "cycle")
            {
                param.cycle = std::stod(cmd_param.second);
            }
            else if (cmd_param.first == "amp")
            {
                param.amp = std::stod(cmd_param.second);
                if (param.amp>0.78) THROW_FILE_AND_LINE("");
            }
            else if (cmd_param.first == "phi0")
            {
                param.phi0 = std::stod(cmd_param.second);
            }
            else if (cmd_param.first == "offset")
            {
                param.offset = std::stod(cmd_param.second);
                if (param.offset!=0) THROW_FILE_AND_LINE("");
            }
            else if (cmd_param.first == "total_time")
            {
                param.total_time = std::stoi(cmd_param.second);
            }
        }

        param.channel.clear();//记得清除
        std::vector<double> init = { 0,0,0 ,0 };
        //初始化channel,完成后每个板卡都是init
        for (Size i = 0; i <= 4; i++)
        {
            param.channel.push_back(init);
        }

        target.param = param;
        /*
                target.option |=
                    Plan::USE_TARGET_POS;
        */
        /*
                target.option |=
        #ifdef WIN32
                    Plan::NOT_CHECK_POS_MIN |
                    Plan::NOT_CHECK_POS_MAX |
                    Plan::NOT_CHECK_POS_CONTINUOUS |
                    Plan::NOT_CHECK_POS_CONTINUOUS_AT_START |
                    Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER |
                    Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER_AT_START |
                    Plan::NOT_CHECK_POS_FOLLOWING_ERROR |
        #endif
                    Plan::NOT_CHECK_VEL_MIN |
                    Plan::NOT_CHECK_VEL_MAX |
                    Plan::NOT_CHECK_VEL_CONTINUOUS |
                    Plan::NOT_CHECK_VEL_CONTINUOUS_AT_START |
                    Plan::NOT_CHECK_VEL_FOLLOWING_ERROR;
        */
    }
    auto MoveCosFC::executeRT(PlanTarget &target)->int
    {
        auto &param = std::any_cast<MoveCosFCParam&>(target.param);
        auto controller = dynamic_cast<aris::control::EthercatController*>(target.controller);

        //获取第一个count时，电机的当前角度位置//
        if (target.count == 1)
        {
            for (Size i = 2; i < 3; ++i)//param.joint_active_vec.size() to 1
            {
                if (param.joint_active_vec[i])
                {
                    param.begin_pos = controller->motionAtAbs(i).actualPos();
                }
            }
            //setting elmo driver max. torque
            auto &ec_mot = static_cast<aris::control::EthercatMotion&>(controller->motionAtPhy(2));
            ec_mot.writePdo(0x6072, 0x00, std::int16_t(5000));
        }
       /*
        if (target.count <= 500)
        {
            param.amp_inc += param.amp / 500;
            param.offset_inc += param.offset / 500;
        }
        */
        param.amp_inc = param.amp;
        param.offset_inc = param.offset ;
        aris::Size total_count{ 1 };
        for (Size i = 2; i < 3; ++i)//param.joint_active_vec.size() to 1
        {
            if (param.joint_active_vec[i])
            {

                aris::Size t_count;
                //长时间扫频实验
                /*
                if(target.count<=1*param.cycle)
                    param.target_pos = param.begin_pos + param.amp_inc*(std::cos(1.0*target.count / param.cycle * 2 * aris::PI +param.phi0)-1) + param.offset_inc;
                else if(target.count<=(1+0.5)*param.cycle)
                    param.target_pos = param.begin_pos + param.amp_inc*(std::cos(1.0*(target.count-param.cycle * 1) / (param.cycle*0.5) * 2 * aris::PI +param.phi0)-1) + param.offset_inc;
                else if(target.count<=(1+0.5+0.1)*param.cycle)
                    param.target_pos = param.begin_pos + param.amp_inc*(std::cos(1.0*(target.count-param.cycle * 1.5) / (param.cycle*0.1) * 2 * aris::PI +param.phi0)-1) + param.offset_inc;
                else if(target.count<=(1+0.5+0.1+0.05)*param.cycle)
                    param.target_pos = param.begin_pos + param.amp_inc*(std::cos(1.0*(target.count-param.cycle * 1.6) / (param.cycle*0.05) * 2 * aris::PI +param.phi0)-1) + param.offset_inc;
                else if(target.count<=(1+0.5+0.1+0.05+0.02)*param.cycle)
                    param.target_pos = param.begin_pos + param.amp_inc*(std::cos(1.0*(target.count-param.cycle * 1.65) / (param.cycle*0.02) * 2 * aris::PI +param.phi0)-1) + param.offset_inc;
                else
                    param.target_pos = param.begin_pos;
                */
                //交互力实验

                if(target.count<=3*param.cycle)
                    param.target_pos = param.begin_pos + param.amp_inc*(std::cos(1.0*target.count / param.cycle * 2 * aris::PI +param.phi0)-1) + param.offset_inc;
                else if(target.count<=(1*3+0.5*2)*param.cycle)
                    param.target_pos = param.begin_pos + param.amp_inc*(std::cos(1.0*(target.count-param.cycle * 1) / (param.cycle*0.5) * 2 * aris::PI +param.phi0)-1) + param.offset_inc;
                else
                    param.target_pos = param.begin_pos;
                controller->motionAtAbs(i).setTargetPos(param.target_pos);
                /*aris::plan::moveAbsolute(target.count, param.begin_pos, param.begin_pos + param.target_pos, param.vel / 1000, param.acc / 1000 / 1000, param.dec / 1000 / 1000, p, v, a, t_count);*/
                /*controller->motionAtAbs(i).setTargetPos(p);*/
                target.model->motionPool().at(i).setMp(param.target_pos);
                /*	total_count = std::max(total_count, t_count);*/
            }
        }
        //3D模型同步
        //if (target.model->solverPool().at(1).kinPos())return -1;

        //采集信号

        int16_t rawData;
        double volToSig = 10.0;
        //电机下标为at(0,1,2),耦合器下标at(3,4),
        //读取PDO，第一参数为index，第二参数为subindex，第三参数读取数据，第四参数为操作位数
        //16位精度
        for (Size i = 5; i <= 5; ++i)//
        {
            //controller->slavePool().at(i).readPdo(0x6000, 0x11, &rawData, 16);
            //param.channel[i - 5][0] = volToSig * rawData / 32767;//2^15-1=32767
            //controller->slavePool().at(i).readPdo(0x6010, 0x11, &rawData, 16);
            //param.channel[i - 5][1] = volToSig * rawData / 32767;
            controller->slavePool().at(i).readPdo(0x6020, 0x11, &rawData, 16);
            param.channel[i - 5][2] = volToSig * rawData / 32767;
            //controller->slavePool().at(i).readPdo(0x6030, 0x11, &rawData, 16);
            //mvj_param->channel[i - 5][3] = volToSig * rawData / 32767;
        }
        for (Size i = 6; i <=6; ++i)//
        {
            controller->slavePool().at(i).readPdo(0x6000, 0x11, &rawData, 16);
            param.channel[i-5][0] = volToSig * rawData / 32767;//2^15-1=32767
            controller->slavePool().at(i).readPdo(0x6010, 0x11, &rawData, 16);
            param.channel[i-5][1] = volToSig * rawData / 32767;
            //controller->slavePool().at(i).readPdo(0x6020, 0x11, &rawData, 16);
            //param.channel[i-5][2] = volToSig * rawData / 32767;
            //controller->slavePool().at(i).readPdo(0x6030, 0x11, &rawData, 16);
            //param.channel[i-5][3] = volToSig * rawData / 32767;
        }
        // 打印 //
        auto &cout = controller->mout();
        if (target.count % 100 == 0)
        {
            cout << "target_pos" << ":" << param.target_pos << " ";
            //cout << "vel" << ":" << param.vel << " ";
            //cout << "acc" << ":" << param.acc << " ";
            //cout << "dec" << ":" << param.dec << " ";

            for (Size i = 2; i < 3; ++i)//param.joint_active_vec.size() to 1
            {
                if (param.joint_active_vec[i])
                {
                    cout << "actualPos" << ":" << controller->motionAtAbs(i).actualPos() << " ";
                    //cout << "actualVel" << ":" << controller->motionAtAbs(i).actualVel() << " ";
                    cout << "actualTor" << ":" << controller->motionAtAbs(i).actualTor() << " ";
                    cout<<param.amp_inc;

                }
            }

            cout << std::endl;
        }

        // log //
        auto &lout = controller->lout();
        for (Size i = 6; i <= 6; ++i)
        {
            lout << param.channel[i - 5][0] << ",";
            lout << param.channel[i - 5][1] << ",";
            //lout << param.channel[i - 5][2] << ",";
            //lout << param.channel[i - 5][3] << ",";
        }

        for (Size i = 5; i <= 5; ++i)
        {
            //lout << param.channel[i - 5][0] << ",";
            //lout << param.channel[i - 5][1] << ",";
            lout << param.channel[i - 5][2] << ",";
            //lout << param.channel[i - 5][3] << ",";
        }
        for (Size i = 2; i < 3; i++)//param.joint_active_vec.size() to 1
        {
            lout << controller->motionAtAbs(i).targetPos() << ",";
            lout << controller->motionAtAbs(i).actualPos() << ",";
            //lout << controller->motionAtAbs(i).actualVel() << ",";
            lout << controller->motionAtAbs(i).actualTor() << ",";
        }
        lout << std::endl;

        return param.total_time - target.count;
    }
    auto MoveCosFC::collectNrt(PlanTarget &target)->void {}
    MoveCosFC::MoveCosFC(const std::string &name) :Plan(name)
    {// CYCLE=10000   TIME=167000
        command().loadXmlStr(
            "<Command name=\"cosfc\">"
            "	<GroupParam>"
            "		<Param name=\"motion_id\" abbreviation=\"m\" default=\"2\"/>"
            "		<Param name=\"amp\" default=\"0.34906585\"/>"
            "		<Param name=\"cycle\" default=\"20000\"/>"
            "		<Param name=\"phi0\" default=\"0\"/>"
            "		<Param name=\"offset\" default=\"0\"/>"
            "		<Param name=\"total_time\" default=\"80000\"/>"
            "		<UniqueParam default=\"check_none\">"
            "			<Param name=\"check_all\"/>"
            "			<Param name=\"check_none\"/>"
            "			<GroupParam>"
            "				<UniqueParam default=\"check_pos\">"
            "					<Param name=\"check_pos\"/>"
            "					<Param name=\"not_check_pos\"/>"
            "					<GroupParam>"
            "						<UniqueParam default=\"check_pos_max\">"
            "							<Param name=\"check_pos_max\"/>"
            "							<Param name=\"not_check_pos_max\"/>"
            "						</UniqueParam>"
            "						<UniqueParam default=\"check_pos_min\">"
            "							<Param name=\"check_pos_min\"/>"
            "							<Param name=\"not_check_pos_min\"/>"
            "						</UniqueParam>"
            "						<UniqueParam default=\"check_pos_continuous\">"
            "							<Param name=\"check_pos_continuous\"/>"
            "							<Param name=\"not_check_pos_continuous\"/>"
            "						</UniqueParam>"
            "						<UniqueParam default=\"check_pos_continuous_at_start\">"
            "							<Param name=\"check_pos_continuous_at_start\"/>"
            "							<Param name=\"not_check_pos_continuous_at_start\"/>"
            "						</UniqueParam>"
            "						<UniqueParam default=\"check_pos_continuous_second_order\">"
            "							<Param name=\"check_pos_continuous_second_order\"/>"
            "							<Param name=\"not_check_pos_continuous_second_order\"/>"
            "						</UniqueParam>"
            "						<UniqueParam default=\"check_pos_continuous_second_order_at_start\">"
            "							<Param name=\"check_pos_continuous_second_order_at_start\"/>"
            "							<Param name=\"not_check_pos_continuous_second_order_at_start\"/>"
            "						</UniqueParam>"
            "						<UniqueParam default=\"check_pos_following_error\">"
            "							<Param name=\"check_pos_following_error\"/>"
            "							<Param name=\"not_check_pos_following_error\"/>"
            "						</UniqueParam>"
            "					</GroupParam>"
            "				</UniqueParam>"
            "				<UniqueParam default=\"check_vel\">"
            "					<Param name=\"check_vel\"/>"
            "					<Param name=\"not_check_vel\"/>"
            "					<GroupParam>"
            "						<UniqueParam default=\"check_vel_max\">"
            "							<Param name=\"check_vel_max\"/>"
            "							<Param name=\"not_check_vel_max\"/>"
            "						</UniqueParam>"
            "						<UniqueParam default=\"check_vel_min\">"
            "							<Param name=\"check_vel_min\"/>"
            "							<Param name=\"not_check_vel_min\"/>"
            "						</UniqueParam>"
            "						<UniqueParam default=\"check_vel_continuous\">"
            "							<Param name=\"check_vel_continuous\"/>"
            "							<Param name=\"not_check_vel_continuous\"/>"
            "						</UniqueParam>"
            "						<UniqueParam default=\"check_vel_continuous_at_start\">"
            "							<Param name=\"check_vel_continuous_at_start\"/>"
            "							<Param name=\"not_check_vel_continuous_at_start\"/>"
            "						</UniqueParam>"
            "						<UniqueParam default=\"check_vel_following_error\">"
            "							<Param name=\"check_vel_following_error\"/>"
            "							<Param name=\"not_check_vel_following_error\"/>"
            "						</UniqueParam>"
            "					</GroupParam>"
            "				</UniqueParam>"
            "			</GroupParam>"
            "		</UniqueParam>"
            "	</GroupParam>"
            "</Command>");
    }


    struct MoveAndAcquireParam
        {
            //运动的
            double begin_pos, target_pos, vel, acc, dec;
            std::vector<bool> joint_active_vec;
            //采集的
            std::vector<std::vector<double>> channel;

        };
	auto MoveAndAcquire::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
			MoveAndAcquireParam param;
		param.joint_active_vec.clear();

		for (auto cmd_param : params)
		{
			if (cmd_param.first == "motion_id")
			{

				param.joint_active_vec.resize(target.model->motionPool().size(), false);
				param.joint_active_vec.at(std::stoi(cmd_param.second)) = true;
			}
			else if (cmd_param.first == "pos")
			{
				param.target_pos = std::stod(cmd_param.second) / 180.0 * PI;
			}
			else if (cmd_param.first == "vel")
			{
				param.vel = std::stod(cmd_param.second);
			}
			else if (cmd_param.first == "acc")
			{
				param.acc = std::stod(cmd_param.second);
			}
			else if (cmd_param.first == "dec")
			{
				param.dec = std::stod(cmd_param.second);
			}
		}
		param.channel.clear();//记得清除
		std::vector<double> init = { 0,0,0 ,0 };
		//初始化channel,完成后每个板卡都是init
		for (Size i = 5; i <= 8; i++)
		{
			param.channel.push_back(init);
		}


		target.param = param;
		//将所有option设置为NOT_CHECK_ENABLE     原sensor中没有注释掉
		//std::fill(target.mot_options.begin(), target.mot_options.end(), Plan::NOT_CHECK_ENABLE);

	}
	auto MoveAndAcquire::executeRT(PlanTarget &target)->int
	{
		auto &param = std::any_cast<MoveAndAcquireParam&>(target.param);
		// 访问主站 //
		auto controller = dynamic_cast<aris::control::EthercatController*>(target.controller);
		//获取第一个count时，电机的当前角度位置//
		if (target.count == 1)
		{
			for (Size i = 0; i < 3; ++i)//param.joint_active_vec.size() to 1
			{
				if (param.joint_active_vec[i])
				{
					param.begin_pos = controller->motionAtAbs(i).actualPos();
					//setting elmo driver max. torque
					auto &ec_mot = static_cast<aris::control::EthercatMotion&>(controller->motionAtPhy(i));
					ec_mot.writePdo(0x6072, 0x00, std::int16_t(1000));
				}
			}
		}
		//梯形轨迹//
		aris::Size total_count{ 1 };
		for (Size i = 0; i < 3; ++i)//param.joint_active_vec.size() to 1
		{
			if (param.joint_active_vec[i])
			{
				double p, v, a;
				aris::Size t_count;
				aris::plan::moveAbsolute(target.count, param.begin_pos, param.begin_pos + param.target_pos,
					param.vel / 1000, param.acc / 1000 / 1000, param.dec / 1000 / 1000, p, v, a, t_count); //规划位置  修改P。   自动count+1?
				controller->motionAtAbs(i).setTargetPos(p);  //和setMP 有点区别   //电机就动了
				//target.model->motionPool().at(i).setMp(p);  //模型
				total_count = std::max(total_count, t_count);
			}
		}
		//3D模型同步
		//if (target.model->solverPool().at(1).kinPos())return -1;
		//采集信号
		int16_t rawData;
		double volToSig = 10.0;
		//电机下标为at(0,1,2),耦合器下标at(3,4),
		//读取PDO，第一参数为index，第二参数为subindex，第三参数读取数据，第四参数为操作位数
		//16位精度
		for (Size i = 5; i <= 8; ++i)//
		{
			controller->slavePool().at(i).readPdo(0x6000, 0x11, &rawData, 16);
			param.channel[i - 5][0] = volToSig * rawData / 32767;//2^15-1=32767
			controller->slavePool().at(i).readPdo(0x6010, 0x11, &rawData, 16);
			param.channel[i - 5][1] = volToSig * rawData / 32767;
			controller->slavePool().at(i).readPdo(0x6020, 0x11, &rawData, 16);
			param.channel[i - 5][2] = volToSig * rawData / 32767;
			controller->slavePool().at(i).readPdo(0x6030, 0x11, &rawData, 16);
			param.channel[i - 5][3] = volToSig * rawData / 32767;
		}

		// 打印 //
		auto &cout = controller->mout();
		if (target.count % 100 == 0)
		{
			//cout << "target_pos" << ":" << param.target_pos << " ";
			//cout << "vel" << ":" << param.vel << " ";
			//cout << "acc" << ":" << param.acc << " ";
			//cout << "dec"  << ":" << param.dec << " ";
			for (Size i = 0; i < 3; ++i)//param.joint_active_vec.size() to 1
			{

					cout << "targetPos" << ":" << controller->motionAtPhy(i).targetPos() << " ";
					//cout << "actualVel" << ":" << controller->motionAtAbs(i).actualVel() << " ";
					cout << "actualPos" << ":" << controller->motionAtPhy(i).actualPos() << " ";

			}
			cout << std::endl;
			//cout << "----------------------------------------------------" << std::endl;
		}

		// log //
		auto &lout = controller->lout();
		for (Size i = 0; i < 3; i++)//param.joint_active_vec.size() to 1
		{
			//lout << controller->motionAtAbs(i).targetPos() << ",";
			//lout << controller->motionAtAbs(i).actualPos() << ",";
			//lout << controller->motionAtAbs(i).actualVel() << ",";
			//lout << controller->motionAtAbs(i).actualTor() << ",";
		}
		for (Size i = 5; i <= 8; ++i)
		{
			lout << param.channel[i - 5][0] << ",";
			//lout << param.channel[i - 5][1] << ",";
			//lout << param.channel[i - 5][2] << ",";
			//lout << param.channel[i - 5][3] << ",";
		}
		lout << std::endl;

		return total_count - target.count;
	}
	auto MoveAndAcquire::collectNrt(PlanTarget &target)->void {}
	MoveAndAcquire::MoveAndAcquire(const std::string &name) :Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"movea\">"
			"	<GroupParam>"
			"		<Param name=\"motion_id\" abbreviation=\"m\" default=\"0\"/>"
			"		<Param name=\"pos\" abbreviation=\"p\" default=\"0\"/>"
			"		<Param name=\"vel\" abbreviation=\"v\" default=\"0.3\"/>"
			"		<Param name=\"acc\" default=\"1\"/>"
			"		<Param name=\"dec\" default=\"1\"/>"
			"	</GroupParam>"
			"</Command>");
	}

    struct MoveJ3Param
    {
        std::vector<double> joint_vel, joint_acc, joint_dec, joint_pos, begin_pos;//绝对的角度值。 其他为0-1最大值的百分比
        std::vector<Size> total_count;

        std::vector<bool> joint_active_vec;
        std::vector<std::vector<double>> channel;

    };
    auto MoveJ3::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
    {
        MoveJ3Param mvj_param;

        mvj_param.begin_pos.clear();
        mvj_param.begin_pos.resize(3, 0.0);
        mvj_param.total_count.resize(3, 0);
        mvj_param.joint_active_vec.clear();
        mvj_param.joint_active_vec.resize(target.model->motionPool().size(), false);
        for (Size i = 0; i <= 3; i++)
        {
            mvj_param.joint_active_vec.at(i) = true;
        }

        // find joint acc/vel/dec/pos
        for (auto cmd_param : params)
        {
            auto c = target.controller;
            double max_pos[3]   {    70.0 / 360 * 2 * PI, 70.0 / 360 * 2 * PI,	160.0 / 360 * 2 * PI             };
            double min_pos[3]   {     -5.0 / 360 * 2 * PI, -5.0 / 360 * 2 * PI,	-5.0 / 360 * 2 * PI             };

            if (cmd_param.first == "joint_acc")
            {
                mvj_param.joint_acc.clear();
                mvj_param.joint_acc.resize(3, 0.0);

                auto acc_mat = target.model->calculator().calculateExpression(cmd_param.second);
                if (acc_mat.size() == 1)std::fill(mvj_param.joint_acc.begin(), mvj_param.joint_acc.end(), acc_mat.toDouble());
                else if (acc_mat.size() == 3) std::copy(acc_mat.begin(), acc_mat.end(), mvj_param.joint_acc.begin());
                else THROW_FILE_AND_LINE("");

                for (int i = 0; i < 3; ++i) mvj_param.joint_acc[i] *= target.controller->motionPool()[i].maxAcc();

                // check value validity //
                for (Size i = 0; i< 3; ++i)
                    if (mvj_param.joint_acc[i] <= 0 || mvj_param.joint_acc[i] > c->motionPool()[i].maxAcc())
                        THROW_FILE_AND_LINE("");
            }
            else if (cmd_param.first == "joint_vel")
            {
                mvj_param.joint_vel.clear();
                mvj_param.joint_vel.resize(3, 0.0);

                auto vel_mat = target.model->calculator().calculateExpression(cmd_param.second);
                if (vel_mat.size() == 1)std::fill(mvj_param.joint_vel.begin(), mvj_param.joint_vel.end(), vel_mat.toDouble());
                else if (vel_mat.size() == 3) std::copy(vel_mat.begin(), vel_mat.end(), mvj_param.joint_vel.begin());
                else THROW_FILE_AND_LINE("");

                for (int i = 0; i < 3; ++i)mvj_param.joint_vel[i] *= target.controller->motionPool()[i].maxVel();

                // check value validity //
                for (Size i = 0; i< 3; ++i)
                    if (mvj_param.joint_vel[i] <= 0 || mvj_param.joint_vel[i] > c->motionPool()[i].maxVel())
                        THROW_FILE_AND_LINE("");
            }
            else if (cmd_param.first == "joint_dec")
            {
                mvj_param.joint_dec.clear();
                mvj_param.joint_dec.resize(3, 0.0);

                auto dec_mat = target.model->calculator().calculateExpression(cmd_param.second);
                if (dec_mat.size() == 1)std::fill(mvj_param.joint_dec.begin(), mvj_param.joint_dec.end(), dec_mat.toDouble());
                else if (dec_mat.size() == 3) std::copy(dec_mat.begin(), dec_mat.end(), mvj_param.joint_dec.begin());
                else THROW_FILE_AND_LINE("");

                for (int i = 0; i < 3; ++i) mvj_param.joint_dec[i] *= target.controller->motionPool()[i].maxAcc();

                // check value validity //
                for (Size i = 0; i< 3; ++i)
                    if (mvj_param.joint_dec[i] <= 0 || mvj_param.joint_dec[i] > c->motionPool()[i].maxAcc())
                        THROW_FILE_AND_LINE("");
            }
            else if (cmd_param.first == "joint_pos")
            {
                mvj_param.joint_pos.clear();
                mvj_param.joint_pos.resize(3, 0.0);
                auto pos_mat = target.model->calculator().calculateExpression(cmd_param.second);
                if (pos_mat.size() == 1)std::fill(mvj_param.joint_pos.begin(), mvj_param.joint_pos.end(), pos_mat.toDouble());
                else if (pos_mat.size() == 3) std::copy(pos_mat.begin(), pos_mat.end(), mvj_param.joint_pos.begin());
                else THROW_FILE_AND_LINE("");

                for (int i = 0; i < 3; ++i) mvj_param.joint_pos[i] *= PI / 180.0;//angle_to_rad

                //肘关节联动
                mvj_param.joint_pos[2] += mvj_param.joint_pos[1];
                // check value validity //
                for (Size i = 0; i< 3; ++i)
                    if (mvj_param.joint_pos[i] <= min_pos[i] || mvj_param.joint_pos[i] > max_pos[i])
                        THROW_FILE_AND_LINE("");
            }
        }
        mvj_param.channel.clear();//记得清除
        std::vector<double> init = { 0,0,0 ,0 };
        //初始化channel,完成后每个板卡都是init
        for (Size i = 0; i <= 4; i++)
        {
            mvj_param.channel.push_back(init);
        }


        target.param = mvj_param;
//原来没注释
        //std::vector<std::pair<std::string, std::any>> ret_value;
        //target.ret = ret_value;
    }
    auto MoveJ3::executeRT(PlanTarget &target)->int
    {
        auto &mvj_param = std::any_cast<MoveJ3Param&>(target.param);
        auto controller = dynamic_cast<aris::control::EthercatController*>(target.controller);

        // 取得起始位置 //
        double p, v, a;
        static Size max_total_count;
        if (target.count == 1)
        {
            // init begin_pos //
            for (Size i = 0; i < 3; ++i)//param.joint_active_vec.size() to 1
            {
                    mvj_param.begin_pos[i] = controller->motionAtAbs(i).actualPos();
                    //setting elmo driver max. torque
                    auto &ec_mot = static_cast<aris::control::EthercatMotion&>(controller->motionAtPhy(i));
                    ec_mot.writePdo(0x6072, 0x00, std::int16_t(1000));
                    //得到总count
                    aris::plan::moveAbsolute(target.count, mvj_param.begin_pos[i], mvj_param.joint_pos[i]
                        , mvj_param.joint_vel[i] / 1000, mvj_param.joint_acc[i] / 1000 / 1000, mvj_param.joint_dec[i] / 1000 / 1000
                        , p, v, a, mvj_param.total_count[i]);
            }
                    max_total_count = *std::max_element(mvj_param.total_count.begin(), mvj_param.total_count.end());
        }

        for (Size i = 0; i < 3 ; ++i)
        {
        double p, v, a;

        aris::plan::moveAbsolute(static_cast<double>(target.count) * mvj_param.total_count[i] / max_total_count,
            mvj_param.begin_pos[i], mvj_param.joint_pos[i],
            mvj_param.joint_vel[i] / 1000, mvj_param.joint_acc[i] / 1000 / 1000, mvj_param.joint_dec[i] / 1000 / 1000,
            p, v, a, mvj_param.total_count[i]);//规划位置  修改P。   第一个参数是规划的当前点  最后一个是计算出到总count
        controller->motionAtAbs(i).setTargetPos(p);  //和setMP 有点区别   //电机就动了
        //target.model->motionPool().at(i).setMp(p);  //模型
        }

        //采集信号
///*
        int16_t rawData;
        double volToSig = 10.0;
        //电机下标为at(0,1,2),耦合器下标at(3,4),
        //读取PDO，第一参数为index，第二参数为subindex，第三参数读取数据，第四参数为操作位数
        //16位精度
        for (Size i = 5; i <= 6; ++i)//
        {
            controller->slavePool().at(i).readPdo(0x6000, 0x11, &rawData, 16);
            mvj_param.channel[i - 5][0] = volToSig * rawData / 32767;//2^15-1=32767
            controller->slavePool().at(i).readPdo(0x6010, 0x11, &rawData, 16);
            mvj_param.channel[i - 5][1] = volToSig * rawData / 32767;
            controller->slavePool().at(i).readPdo(0x6020, 0x11, &rawData, 16);
            mvj_param.channel[i - 5][2] = volToSig * rawData / 32767;
            //controller->slavePool().at(i).readPdo(0x6030, 0x11, &rawData, 16);
            //mvj_param->channel[i - 5][3] = volToSig * rawData / 32767;
        }

        // 打印 //
        auto &cout = controller->mout();
        if (target.count % 100 == 0)
        {
            //cout << "target_pos" << ":" << param.target_pos << " ";
            //cout << "vel" << ":" << param.vel << " ";
            //cout << "acc" << ":" << param.acc << " ";
            //cout << "dec"  << ":" << param.dec << " ";
            ///*
            for (Size i = 0; i < 3; ++i)//param.joint_active_vec.size() to 1
            {

                    cout << "targetPos" << ":" << controller->motionAtSla(i).targetPos() << " ";
                    //cout << "actualVel" << ":" << controller->motionAtSla(i).actualVel() << " ";
                    cout << "actualPos" << ":" << controller->motionAtSla(i).actualPos() << " ";
                    cout << "TorqueData" << ":" << std::setw(6) << mvj_param.channel[0][i] << "  ";

            }
            cout << std::endl;
            for (Size i = 6; i <= 6; ++i)
            {//肘关节 肌电
                cout << mvj_param.channel[i - 5][0] << ",";
                cout << mvj_param.channel[i - 5][1] << ",";
            }
            cout << std::endl;
            //*/
            //cout << "----------------------------------------------------" << std::endl;
        }

        // log //
        auto &lout = controller->lout();
        for (Size i = 0; i < 3; i++)//param.joint_active_vec.size() to 1
        {
            lout << controller->motionAtSla(i).targetPos() << ",";
            lout << controller->motionAtSla(i).actualPos() << ",";
            //就这一句有问题 //lout << controller->motionAtSla(i).actualVel() << ",";
            lout << controller->motionAtSla(i).actualTor() << ",";
        }
        for (Size i = 5; i <= 6; ++i)
        {
            lout << mvj_param.channel[i - 5][0] << ",";
            lout << mvj_param.channel[i - 5][1] << ",";
            lout << mvj_param.channel[i - 5][2] << ",";
            //lout << param.channel[i - 5][3] << ",";
        }
        lout << std::endl;
//*/

        return max_total_count == 0 ? 0 : max_total_count - target.count;
    }
    auto MoveJ3::collectNrt(PlanTarget &target)->void {}
    MoveJ3::MoveJ3(const std::string &name) :Plan(name)
    {
        command().loadXmlStr(
            "<Command name=\"movej3\">"
            "	<GroupParam>"
            "		<Param name=\"joint_pos\" abbreviation=\"p\" default=\"{0,0,0}\"/>"
            "		<Param name=\"joint_acc\" abbreviation=\"a\" default=\"0.3\"/>"
            "		<Param name=\"joint_vel\" abbreviation=\"v\" default=\"0.2\"/>"
            "		<Param name=\"joint_dec\" abbreviation=\"d\" default=\"0.3\"/>"
            "	</GroupParam>"
            "</Command>");
    }


    auto EXO_IK( std::vector<double> &tar_pos,std::vector<double> &joint_ang)
{
    double px=tar_pos[0];
    double py=tar_pos[1];
    double pz=tar_pos[2];
    double r;

    r = sqrt(px * px + py * py + pz * pz);
    if (px >= 0.0)
    {
      joint_ang[0] = atan2(py, px);
      joint_ang[2] = PI - acos((0.110036 - r * r) / 0.10712);
      joint_ang[1] = asin(pz / r) - acos((0.025164 + r * r) / (0.52 * r));
    }
    else
    {
      joint_ang[0] = (PI + atan2(-py, -px));
      joint_ang[2] = PI - acos((0.110036 - r * r) / 0.10712);
      joint_ang[1] = asin(pz / r) - acos((0.025164 + r * r ) / (0.52 * r));
    }
}

    struct MoveCircleParam
    {
        std::vector<double>  Center_pos,target_pos,target_angle,begin_pos,last_angle;
        std::vector<double>  joint_pos,joint_ang ;//梯形规划用
        std::vector<Size> total_count;//梯形规划用
        double Radius;
        Size Period ,T_count;
        //采集的
        //std::vector<std::vector<double>> channel;
    };
    auto MoveCircle::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
    {
        MoveCircleParam mvj_param;
        mvj_param.total_count.clear();
        mvj_param.total_count.resize(3, 0);
        mvj_param.begin_pos.clear();
        mvj_param.begin_pos.resize(3, 0.0);
        mvj_param.joint_ang.clear();
        mvj_param.joint_ang.resize(3, 0.0);
        mvj_param.joint_pos.clear();
        mvj_param.joint_pos.resize(3, 0.0);
        mvj_param.last_angle.clear();
        mvj_param.last_angle.resize(3, 0.0);
        mvj_param.target_angle.clear();
        mvj_param.target_angle.resize(3, 0.0);
        mvj_param.Center_pos.clear();
        mvj_param.Center_pos.resize(3, 0.0);
        mvj_param.Radius=0;
        mvj_param.T_count=0;

        // find params
        for (auto cmd_param : params)
        {
            auto c = target.controller;

            if (cmd_param.first == "Center_pos")
            {
         // check value validity //
                auto acc_mat = target.model->calculator().calculateExpression(cmd_param.second);
                if (acc_mat.size() == 3) std::copy(acc_mat.begin(), acc_mat.end(), mvj_param.Center_pos.begin());
                else THROW_FILE_AND_LINE("");


            }
            else if (cmd_param.first == "Radius")
            {
                auto vel_mat = target.model->calculator().calculateExpression(cmd_param.second);
                if (vel_mat.size() == 1)  mvj_param.Radius=vel_mat.toDouble();
                else THROW_FILE_AND_LINE("");

                // check value validity //

                if (mvj_param.Radius > 0.3)
                        THROW_FILE_AND_LINE("");
            }
            else if (cmd_param.first == "Period")
            {
                auto dec_mat = target.model->calculator().calculateExpression(cmd_param.second);
                if (dec_mat.size() == 1)  mvj_param.Period = std::stoi(cmd_param.second);
                else THROW_FILE_AND_LINE("");


                if (mvj_param.Period <= 0 )
                     THROW_FILE_AND_LINE("");
            }

        }

        // check value validity //
        for (Size i = 0; i< 3; ++i)
            if ( mvj_param.Center_pos[i]+mvj_param.Radius > 0.466)
                THROW_FILE_AND_LINE("");


        //mvj_param.channel.clear();//记得清除
        //std::vector<double> init = { 0,0,0 ,0 };
        //初始化channel,完成后每个板卡都是init
        for (Size i = 0; i <= 4; i++)
        {
            //mvj_param.channel.push_back(init);
        }


        target.param = mvj_param;
//原来没注释
        //std::vector<std::pair<std::string, std::any>> ret_value;
        //target.ret = ret_value;
        target.option |=
        Plan::USE_TARGET_POS |
        Plan::NOT_CHECK_VEL_CONTINUOUS|
        Plan::NOT_CHECK_VEL_CONTINUOUS_AT_START |
        Plan::NOT_CHECK_POS_CONTINUOUS |
        Plan::NOT_CHECK_POS_CONTINUOUS_AT_START |
        Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER |
        Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER_AT_START |
        Plan::NOT_CHECK_POS_FOLLOWING_ERROR |
        Plan::NOT_CHECK_VEL_FOLLOWING_ERROR;
    }
    auto MoveCircle::executeRT(PlanTarget &target)->int
    {
        auto &mvj_param = std::any_cast<MoveCircleParam&>(target.param);
        auto controller = dynamic_cast<aris::control::EthercatController*>(target.controller);
        auto &cout = controller->mout();
        double px=0.0;
        double py=0.0;
        double pz=0.0;
        // 取得起始位置 //

        static Size max_total_count = mvj_param.Period;
        if (target.count == 1)
        {
        cout << "进入" << std::endl;
            // init begin_pos //
            for (Size i = 0; i < 3; ++i)//param.joint_active_vec.size() to 1
            {
                    mvj_param.begin_pos[i] = controller->motionAtAbs(i).actualPos();
                    //setting elmo driver max. torque
                    auto &ec_mot = static_cast<aris::control::EthercatMotion&>(controller->motionAtPhy(i));
                    ec_mot.writePdo(0x6072, 0x00, std::int16_t(2000));
             }

            mvj_param.joint_pos=mvj_param.Center_pos;
            mvj_param.joint_pos[0]+=mvj_param.Radius;//初始坐标
            //joint_pos为初始坐标，再IK转换为角度
            EXO_IK(mvj_param.joint_pos,mvj_param.joint_ang);
            mvj_param.last_angle=mvj_param.joint_ang;
            mvj_param.last_angle[2]+=mvj_param.last_angle[1];
/*
            //梯形预规划
            for (Size i = 0; i < 3 ; ++i)
            {
            double p, v, a;
            //得到总count
            aris::plan::moveAbsolute(target.count, mvj_param.begin_pos[i], mvj_param.joint_ang[i]
                , 0.2/ 1000, 0.2 / 1000 / 1000, 0.2 / 1000 / 1000
                , p, v, a, mvj_param.total_count[i]);
            }
             mvj_param.T_count = *std::max_element(mvj_param.total_count.begin(), mvj_param.total_count.end());
             cout << "T_count" << mvj_param.T_count << std::endl;
             */
        }
        max_total_count = mvj_param.Period + mvj_param.T_count;
        //规划梯形
        if (target.count< mvj_param.T_count)
            {

                for (Size i = 0; i < 3 ; ++i)
                {
                double p, v, a;
                aris::plan::moveAbsolute(static_cast<double>(target.count) * mvj_param.total_count[i] / mvj_param.T_count,
                mvj_param.begin_pos[i], mvj_param.joint_ang[i],
                0.2 / 1000, 0.2 / 1000 / 1000, 0.2/ 1000 / 1000,
                p, v, a, mvj_param.total_count[i]);//规划位置  修改P。   第一个参数是规划的当前点  最后一个是计算出到总count
                mvj_param.target_pos[i]=p;
                }

            if (target.count % 100 == 0) cout << "直线" << std::endl;

            }
        else if(target.count<= max_total_count)
        {
        if (target.count % 100 == 0) cout << "圆弧" << std::endl;

        //规划位置
        py = mvj_param.Center_pos[1];
        px = mvj_param.Center_pos[0] + mvj_param.Radius * std::cos(target.count*2*PI/mvj_param.Period);
        pz = mvj_param.Center_pos[2] + mvj_param.Radius * std::sin(target.count*2*PI/mvj_param.Period);
        mvj_param.target_pos={px,py,pz};
        ///*
        //IK
        EXO_IK(mvj_param.target_pos,mvj_param.target_angle);

        }

        //*/
        //驱动
        mvj_param.target_angle[2]=mvj_param.target_angle[2]+mvj_param.target_angle[1];//联动
        for (Size i = 0; i < 3 ; ++i)
            {
            if (fabs(mvj_param.target_angle[i]-mvj_param.last_angle[i])>=0.1)//关节空间
                THROW_FILE_AND_LINE("");
            else
                {controller->motionAtAbs(i).setTargetPos(mvj_param.target_angle[i]);}  //和setMP 有点区别   //电机就动了
            }
        mvj_param.last_angle=mvj_param.target_angle;
        //cout << "驱动" << std::endl;
        //采集信号
/*
        int16_t rawData;
        double volToSig = 10.0;
        //电机下标为at(0,1,2),耦合器下标at(3,4),
        //读取PDO，第一参数为index，第二参数为subindex，第三参数读取数据，第四参数为操作位数
        //16位精度
        for (Size i = 5; i <= 5; ++i)//
        {
            controller->slavePool().at(i).readPdo(0x6000, 0x11, &rawData, 16);
            mvj_param.channel[i - 5][0] = volToSig * rawData / 32767;//2^15-1=32767
            controller->slavePool().at(i).readPdo(0x6010, 0x11, &rawData, 16);
            mvj_param.channel[i - 5][1] = volToSig * rawData / 32767;
            controller->slavePool().at(i).readPdo(0x6020, 0x11, &rawData, 16);
            mvj_param.channel[i - 5][2] = volToSig * rawData / 32767;
            //controller->slavePool().at(i).readPdo(0x6030, 0x11, &rawData, 16);
            //mvj_param->channel[i - 5][3] = volToSig * rawData / 32767;
        }
     */
        // 打印 //

        if (target.count % 100 == 0)
        {
            //cout << "target_pos" << ":" << param.target_pos << " ";
            //cout << "vel" << ":" << param.vel << " ";
            //cout << "acc" << ":" << param.acc << " ";
            //cout << "dec"  << ":" << param.dec << " ";
            for (Size i = 0; i < 3; ++i)//param.joint_active_vec.size() to 1
            {
                    cout << "targetPos" << ":" << controller->motionAtSla(i).targetPos() << " ";
                    cout << "actualPos" << ":" << controller->motionAtSla(i).actualPos() << " ";
                    //cout << "TorqueData" << ":" << std::setw(6) << mvj_param.channel[0][i] << "  ";
            }
            cout << std::endl;
            //cout << "----------------------------------------------------" << std::endl;
        }

        // log //
        auto &lout = controller->lout();
        for (Size i = 0; i < 3; i++)//param.joint_active_vec.size() to 1
        {
            lout << controller->motionAtSla(i).targetPos() << ",";
            //lout << controller->motionAtSla(i).actualPos() << ",";
            //就这一句有问题 //lout << controller->motionAtSla(i).actualVel() << ",";
            //lout << controller->motionAtSla(i).actualTor() << ",";
        }

        lout << std::endl;

        return max_total_count - target.count;
    }
    auto MoveCircle::collectNrt(PlanTarget &target)->void {}
    MoveCircle::MoveCircle(const std::string &name) :Plan(name)
    {
        command().loadXmlStr(
            "<Command name=\"movecc\">"
            "	<GroupParam>"
            "		<Param name=\"Center_pos\" abbreviation=\"c\" default=\"{0.28,0.05,0.28}\"/>"
            "		<Param name=\"Radius\" abbreviation=\"r\" default=\"0.05\"/>"
            "		<Param name=\"Period\" abbreviation=\"p\" default=\"20000\"/>"
            "	</GroupParam>"
            "</Command>");
    }


    double  TorSensor2Offset = 2.478;
	auto fx_TorqueNorm(double x)
	{
        const double threshold = 0.020;
        //const double peak = 0.2;
        x -= TorSensor2Offset;
        if ((x>0)&&(x< threshold))
			x = 0;
        if ((x<0)&&(x> -0.5*threshold))
            x = 0;
        //else if (std::fabs(x) > peak)
        //    x = peak;
        //x = x / peak;//归一化
		return x;
	}
//加&表示改变原变量
    double fx_EnhanceData(double x)
    {
        const double threshold = 0.5;
        const double peak = 1;
        //削顶
        if ( x > peak )
        {
            x = peak;
        }
        else if( x < -peak )
        {
            x = -peak;
        }
        //去阈值
        if ( x > 0 )//负的交互力容易 正的难
        {
            x -= 0*threshold;
            if( x < 0 )
                x = 0;
        }
        else if( x < 0 )
        {
            x += 1.0 * threshold;
            if (x >0)
            x = 0;
        }
        //归一化
        x /= (peak-threshold)*1;
        if (x<0) x*=1.0;
        //自平方
        //x = x * std::fabs(x);
        return x*2;
    }


    auto fx_Filter_lowpass_5(std::deque<double> y, std::deque<double> &y_filtered)//y 最近三个采样值 y_filtered 上两个滤波值  返回第三位当前滤波值
	{
        const std::vector<double> a = { 1,-1.97778648,0.97803051 };
        const std::vector<double> b = { 0.0000610061788,0.000122012358,0.0000610061788};
        double filtered_data=0.0;
        filtered_data = b[0] * y.at(2) + b[1] * y.at(1) + b[2] * y.at(0) - a[1] * y_filtered.at(1) - a[2] * y_filtered.at(0);
		y_filtered.push_back(filtered_data);
	}
	//T=A*sin(x)+B
	double Gs = 0;//重力力补偿系数,0~1缓慢变化
    double fx_GravityComp(std::deque<double> pos, std::deque<double> y_filtered)
	{//up - k1    down - k2
        const double k1 = 4.3, k2 = 1.59;//k1=4
		double angle, torquedata, GC;
		double Gs_step = 0;
        //angle = pos.back() * 180.0 / PI ;
		Gs_step = (pos.at(2) - pos.at(1))   / 0.0561 ;//vel*1000 /backlash/1000
		Gs = Gs + Gs_step;
		if (Gs > 1) Gs = 1;
		if (Gs < 0) Gs = 0;


        GC = ((1 - Gs) * k2 + k1 * Gs)*sin(pos.at(2));

        torquedata = y_filtered.back() - GC;//原来加
		return torquedata;
	}
    double Ks = 0;//摩擦力补偿系数,-1~1缓慢变化
    double Ksf = 0;//摩擦力补偿系数f,0~1缓慢变化
    const double backlash =0.056 ;
    double fx_FrictionComp(std::deque<double> pos, double torque,double FriComp_last)//输入三个时刻的position,扭矩值，返回摩擦力补偿后的扭矩值
	{
		//friction function a1+b1*exp(-c1*x)+d1*qd
        const double a1 = -2.26, b1 = 2, c1 = 2, d1 = 0.669;
        const double a2 = -0.4966, b2 = 0.076, c2 = 0.0002, d2 = 0.3993;
		//actan
        const double k1 = 0.7420, p1 = 2.0749;
		const double k2 = 0.7910, p2 = 1.8053;
		double B_up , B_down ;
        double torquedata, FC;
        double K_step2 = 0;//09 04
		double vel;
		vel = (pos.at(2) - pos.at(1)) * 1000;
		K_step2 = 2 * fabs(vel) / backlash / 1000;
		//速度阈值
		if ((vel < 0.007)&(vel >= 0))
			vel = 0.007;
		if ((vel > -0.007)&(vel < 0))
			vel = -0.007;

		//计算摩擦力值
		if (vel >= 0)
		{
			B_up   = a1 + b1 * exp(-1 * c1 * vel) + d1 * vel;
			B_down = a2 + b2 * exp(-1 * c2 *(-0.007))+ d2 * (-0.007); 
		}
		else
		{
			B_down = a2 + b2 * exp(-1 * c2 * vel) + d2 * vel; //绝对值
			B_up   = a1 + b1 * exp(-1 * c1 *0.007) + d1 * 0.007;
		}
			
		//摩擦力阈值
		if (B_up > 2)				B_up = 2;
		if (B_down < -2)			B_down = -2;

		//连续变化
		if (vel > 0) //当前正向
		{
			Ks = Ks + K_step2;				//Kf
			if (Ks >= 1)				Ks = 1;
			//plot(t, k*atan(c*(t + 1)), '--')
			//plot(-t, k*atan(c*(t + 1)), '--')
            Ksf = k1 * atan(p1*(Ks + 1));	//Kf~
            if (Ksf <= -0.99)			Ksf = -1;
            if (Ksf >= 0.9909)			Ksf = 1;

            FC = B_down + Ksf * (B_up - B_down);
		}
        else if(vel < 0) // 当前反向
		{ 
			Ks = Ks - K_step2;

			if (Ks <= -1)				Ks = -1;
            Ksf = k2 * atan(p2*(-Ks + 1));
            if (Ksf <= -0.99)		Ksf = -1;
            if (Ksf >= 0.99)		Ksf = 1;

            FC = B_up + Ksf * (B_down - B_up);
		}		
		else
		FC = FriComp_last;

		//补偿
        torquedata = torque  -FC;//原来加
		return torquedata;
	}
	std::deque<double> ElbowTorque;
	std::deque<double> ElbowTorque_Filtered;
	std::deque<double> ElbowPosition;
	struct ElbowTorqueControlParam
	{
        std::vector<double> K_vel, actual_pos , target_vel,target_pos;//绝对的角度值。 其他为0-1最大值的百分比
		int total_time;
        double FriComp_last, Vel_last;
		std::vector<bool> joint_active_vec;
		std::vector<std::vector<double>> channel;

	};
	auto ElbowTorqueControl::prepairNrt(const std::map<std::string, std::string> &params, PlanTarget &target)->void
	{
		ElbowTorqueControlParam etc_param;
		//初始化
		ElbowTorque.clear();
		ElbowTorque_Filtered.clear();
		ElbowPosition.clear();
		Ks = 0;
        Ksf = 0;
        Gs = 0;
        etc_param.Vel_last=0;
        etc_param.FriComp_last = 0.0;
		etc_param.actual_pos.resize(3, 0.0);
		etc_param.target_pos.resize(3, 0.0);
		etc_param.target_vel.resize(3, 0.0);
		etc_param.K_vel.resize(3, 0.0);
		etc_param.joint_active_vec.clear();
		etc_param.joint_active_vec.resize(target.model->motionPool().size(), false);
		for (Size i = 0; i <= 2; i++)
		{
			etc_param.joint_active_vec.at(i) = false;
		}
        etc_param.joint_active_vec.at(2) = true;
		// find joint vel/time
		for (auto cmd_param : params)
		{
			auto c = target.controller;
            if (cmd_param.first == "K_vel")
			{

				auto vel_mat = target.model->calculator().calculateExpression(cmd_param.second);
				if (vel_mat.size() == 1)std::fill(etc_param.K_vel.begin(), etc_param.K_vel.end(), vel_mat.toDouble());
				else if (vel_mat.size() == 3) std::copy(vel_mat.begin(), vel_mat.end(), etc_param.K_vel.begin());
				else THROW_FILE_AND_LINE("");

                //for (int i = 0; i < 3; ++i) etc_param.K_vel[i] *= target.controller->motionPool()[i].maxVel();

				// check value validity //
				for (Size i = 0; i < 3; ++i)
                    //if (etc_param.K_vel[i] <= 0 || etc_param.K_vel[i] > c->motionPool()[i].maxVel())
                    if (etc_param.K_vel[i] <= 0 || etc_param.K_vel[i] > 1)
						THROW_FILE_AND_LINE("");
			}
			else if (cmd_param.first == "total_time")
			{
				etc_param.total_time = std::stoi(cmd_param.second);
			}
		}
		etc_param.channel.clear();//记得清除
		std::vector<double> init = { 0,0,0 ,0 };
		//初始化channel,完成后每个板卡都是init
		for (Size i = 0; i <= 4; i++)
		{
			etc_param.channel.push_back(init);
		}
		target.param = etc_param;
        target.option |=
        Plan::USE_TARGET_POS |
        Plan::NOT_CHECK_VEL_CONTINUOUS|
        Plan::NOT_CHECK_VEL_CONTINUOUS_AT_START |
        Plan::NOT_CHECK_POS_CONTINUOUS |
        Plan::NOT_CHECK_POS_CONTINUOUS_AT_START |
        Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER |
        Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER_AT_START |
        Plan::NOT_CHECK_POS_FOLLOWING_ERROR |
        Plan::NOT_CHECK_VEL_FOLLOWING_ERROR;
		//原来没注释
							//std::vector<std::pair<std::string, std::any>> ret_value;
							//target.ret = ret_value;
/*
        target.option |=Plan::USE_TARGET_POS|
        Plan::USE_TARGET_POS |
        Plan::NOT_CHECK_VEL_CONTINUOUS|
        Plan::NOT_CHECK_VEL_CONTINUOUS_AT_START |
        Plan::NOT_CHECK_VEL_FOLLOWING_ERROR|
             Plan::NOT_CHECK_POS_CONTINUOUS |
             Plan::NOT_CHECK_POS_CONTINUOUS_AT_START |
             Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER |
             Plan::NOT_CHECK_POS_CONTINUOUS_SECOND_ORDER_AT_START |
             Plan::NOT_CHECK_POS_FOLLOWING_ERROR ;
             */
	}
	auto ElbowTorqueControl::executeRT(PlanTarget &target)->int
	{
		auto &param = std::any_cast<ElbowTorqueControlParam&>(target.param);
		auto controller = dynamic_cast<aris::control::EthercatController*>(target.controller);
        const double maxstep = 1 / 5000.0;//最大步长
        double Human_Tortemp = 0.0,G_VALUE=0.0,F_VALUE=0.0;
		//电机扭矩使能初始化
		if (target.count == 1)
		{
			// init begin_pos //
			for (Size i = 2; i < 3; ++i)//param.joint_active_vec.size() to 1
			{								//setting elmo driver max. torque
				auto &ec_mot = static_cast<aris::control::EthercatMotion&>(controller->motionAtPhy(i));
                ec_mot.writePdo(0x6072, 0x00, std::int16_t(5000));//原始1000
			}
		}

		// 取得当前位置 //
		for (Size i = 2; i < 3; ++i)//param.joint_active_vec.size() to 1
		{
			param.actual_pos[i] = controller->motionAtAbs(i).actualPos();
		}

		//采集信号
		int16_t rawData;
		double volToSig = 10.0;
		//电机下标为at(0,1,2),耦合器下标at(3,4),
		//读取PDO，第一参数为index，第二参数为subindex，第三参数读取数据，第四参数为操作位数
		//16位精度 -10~+10
		for (Size i = 5; i <= 6; ++i)//
		{
			controller->slavePool().at(i).readPdo(0x6000, 0x11, &rawData, 16);
			param.channel[i - 5][0] = volToSig * rawData / 32767;//2^15-1=32767
			controller->slavePool().at(i).readPdo(0x6010, 0x11, &rawData, 16);
			param.channel[i - 5][1] = volToSig * rawData / 32767;
			controller->slavePool().at(i).readPdo(0x6020, 0x11, &rawData, 16);
			param.channel[i - 5][2] = volToSig * rawData / 32767;
			//controller->slavePool().at(i).readPdo(0x6030, 0x11, &rawData, 16);
			//mvj_param->channel[i - 5][3] = volToSig * rawData / 32767;
		}
		//初始
		if (target.count < 3)
		{//初始两次
            ElbowTorque.push_back((param.channel[0][2]- 2.478)*24.2589);
            ElbowTorque_Filtered.push_back((param.channel[0][2]- 2.478)*24.2589);
			ElbowPosition.push_back(param.actual_pos[2]);

		}

        double ElbowTorque_G = (param.channel[0][2]- 2.478)*24.2589;
        double ElbowTorque_GF = (param.channel[0][2]- 2.478)*24.2589;
        double Human_Tor = 0.0;
        //正式开始 滤波
		if (target.count > 2)
        {
            ElbowTorque.push_back((param.channel[0][2]- 2.478)*24.2589);
			fx_Filter_lowpass_5(ElbowTorque, ElbowTorque_Filtered);
			ElbowPosition.push_back(param.actual_pos[2]);
			//此时各个队列都长3
			ElbowTorque.pop_front();
			ElbowTorque_Filtered.pop_front();
            //长2 访问ElbowTorque_Filtered.at(1)得到当前扭矩

		//重力补偿 
			ElbowTorque_G = fx_GravityComp(ElbowPosition,ElbowTorque_Filtered);
		//摩擦力补偿
			ElbowTorque_GF = fx_FrictionComp(ElbowPosition, ElbowTorque_G, param.FriComp_last);
		//再滤个波？
		
			//计算交互力
            Human_Tor = 0.0;
            param.target_vel[2]=0.0;
            if (fx_TorqueNorm(ElbowTorque_GF)!=0.0)//大于阈值
            {
                Human_Tortemp = ElbowTorque_GF;//交互力
                Human_Tor = fx_EnhanceData(Human_Tortemp);//去阈值 自平方
            }

            param.target_vel[2] = param.K_vel[2] * (-1)* Human_Tor * maxstep;//速度值(=K*maxVel)
            //限制步长  根据上一个位置增量（速度
            if ((param.target_vel[2]-param.Vel_last)>0.0002)
                param.target_vel[2]=param.Vel_last+0.0002;
            if ((param.target_vel[2]-param.Vel_last)<-0.0002)
                param.target_vel[2]=param.Vel_last-0.0002;
            //连续同向
            if(param.target_vel[2]*param.Vel_last<0)//反向置零
                param.target_vel[2]=0.0;
        double target_pos=param.actual_pos[2];
		// 目标位置 //
        if (target.count > 499)
        {
			for (Size i = 2; i < 3; ++i)//param.joint_active_vec.size() to 1
			{			
                target_pos = ElbowPosition.at(2) + param.target_vel[2];//上一时刻目标位置+增量?  当前

                if (target_pos>(130.01 / 360 * 2 * PI))
                    THROW_FILE_AND_LINE("");
                if (target_pos<(-1.01 / 360 * 2 * PI))
                    THROW_FILE_AND_LINE("");
                controller->motionAtAbs(2).setTargetPos(target_pos);//驱动
			}
        }

            param.Vel_last=param.target_vel[2];
            param.FriComp_last = ElbowTorque_G-ElbowTorque_GF;
            //注释
            //ElbowPosition.pop_back();//不注释好像不会漂移 但更难拖动
            //ElbowPosition.push_back(target_pos);//更新当前位置使之为目标位置
            ElbowPosition.pop_front();//保持队长为2
        }

		// 打印 //
		auto &cout = controller->mout();
        if (target.count % 100 == 0)
		{
			//cout << "target_pos" << ":" << param.target_pos << " ";
			//cout << "vel" << ":" << param.vel << " ";
			//cout << "acc" << ":" << param.acc << " ";
			//cout << "dec"  << ":" << param.dec << " ";
			///*
			for (Size i = 2; i < 3; ++i)//param.joint_active_vec.size() to 1
            {
				//cout << "actualVel" << ":" << controller->motionAtSla(i).actualVel() << " ";
                cout << "actualPos" << ":" << std::setw(4) << controller->motionAtSla(i).actualPos() << " ";
                cout << "incPos" << ":" << std::setw(4) << param.target_vel[2] << " ";
                //cout << "TorqueData" << ":" << std::setw(6) << param.channel[0][i] << "  ";
                cout << "SensorV" << ":" << std::setw(4) << param.channel[0][2] << "  ";
                cout << "SensorTor" << ":" << std::setw(4) <<  Human_Tortemp;
                cout << "G" << ":" << std::setw(4) << ElbowTorque.at(0) - ElbowTorque_G << "  ";
                cout << "F" << ":" << std::setw(4) << ElbowTorque_G - ElbowTorque_GF << "  ";
                cout << "HumanTor" << ":" << std::setw(4) <<  Human_Tortemp;
                cout << "HumanTor_En" << ":" << std::setw(4) << Human_Tor << "  ";
                cout << "Ks" << ":" << std::setw(4) <<  Ks;
			}
			cout << std::endl;
			//cout << "----------------------------------------------------" << std::endl;
		}

		// log //
		auto &lout = controller->lout();
		for (Size i = 2; i < 3; i++)//param.joint_active_vec.size() to 1
		{
            //lout << controller->motionAtSla(i).targetPos() << ",";
			lout << controller->motionAtSla(i).actualPos() << ",";
            lout << param.target_vel[2] << ",";
			//就这一句有问题 //lout << controller->motionAtSla(i).actualVel() << ",";
            //lout << controller->motionAtSla(i).actualTor() << ",";
		}
		for (Size i = 5; i <= 5; ++i)
		{
            lout << param.channel[i - 5][2] << ",    ";
            lout << ElbowTorque.at(0) - ElbowTorque_G << ",    " ;
            lout << ElbowTorque_G - ElbowTorque_GF << ",    ";
            lout << ElbowTorque_Filtered.back() << ",    ";
            lout << ElbowPosition.back() << ",    ";
            lout << param.FriComp_last << ",    ";
            lout << Ks << ",    ";
            lout << Human_Tor << ",    ";
            lout << Human_Tortemp <<  ",    ";
		}
		for (Size i = 6; i <= 6; ++i)
		{
			lout << param.channel[i - 5][0] << ",    ";
			lout << param.channel[i - 5][1] << ",    ";

		}
		lout << std::endl;
		return param.total_time - target.count;
	}
	auto ElbowTorqueControl::collectNrt(PlanTarget &target)->void {}
	ElbowTorqueControl::ElbowTorqueControl(const std::string &name) :Plan(name)
	{
		command().loadXmlStr(
			"<Command name=\"etc\">"
			"	<GroupParam>"
            "		<Param name=\"K_vel\" abbreviation=\"v\" default=\"1\"/>"
            "		<Param name=\"total_time\" abbreviation=\"t\" default=\"50000\"/>"
            "		<UniqueParam default=\"check_all\">"
            "			<Param name=\"check_all\"/>"
            "			<Param name=\"check_none\"/>"
            "			<GroupParam>"
            "				<UniqueParam default=\"check_pos\">"
            "					<Param name=\"check_pos\"/>"
            "					<Param name=\"not_check_pos\"/>"
            "					<GroupParam>"
            "						<UniqueParam default=\"check_pos_max\">"
            "							<Param name=\"check_pos_max\"/>"
            "							<Param name=\"not_check_pos_max\"/>"
            "						</UniqueParam>"
            "						<UniqueParam default=\"check_pos_min\">"
            "							<Param name=\"check_pos_min\"/>"
            "							<Param name=\"not_check_pos_min\"/>"
            "						</UniqueParam>"
            "						<UniqueParam default=\"not_check_pos_continuous\">"
            "							<Param name=\"check_pos_continuous\"/>"
            "							<Param name=\"not_check_pos_continuous\"/>"
            "						</UniqueParam>"
            "						<UniqueParam default=\"not_check_pos_continuous_at_start\">"
            "							<Param name=\"check_pos_continuous_at_start\"/>"
            "							<Param name=\"not_check_pos_continuous_at_start\"/>"
            "						</UniqueParam>"
            "						<UniqueParam default=\"not_check_pos_continuous_second_order\">"
            "							<Param name=\"check_pos_continuous_second_order\"/>"
            "							<Param name=\"not_check_pos_continuous_second_order\"/>"
            "						</UniqueParam>"
            "						<UniqueParam default=\"not_check_pos_continuous_second_order_at_start\">"
            "							<Param name=\"check_pos_continuous_second_order_at_start\"/>"
            "							<Param name=\"not_check_pos_continuous_second_order_at_start\"/>"
            "						</UniqueParam>"
            "						<UniqueParam default=\"check_pos_following_error\">"
            "							<Param name=\"check_pos_following_error\"/>"
            "							<Param name=\"not_check_pos_following_error\"/>"
            "						</UniqueParam>"
            "					</GroupParam>"
            "				</UniqueParam>"
            "				<UniqueParam default=\"check_vel\">"
            "					<Param name=\"check_vel\"/>"
            "					<Param name=\"not_check_vel\"/>"
            "					<GroupParam>"
            "						<UniqueParam default=\"check_vel_max\">"
            "							<Param name=\"check_vel_max\"/>"
            "							<Param name=\"not_check_vel_max\"/>"
            "						</UniqueParam>"
            "						<UniqueParam default=\"check_vel_min\">"
            "							<Param name=\"check_vel_min\"/>"
            "							<Param name=\"not_check_vel_min\"/>"
            "						</UniqueParam>"
            "						<UniqueParam default=\"not_check_vel_continuous\">"
            "							<Param name=\"check_vel_continuous\"/>"
            "							<Param name=\"not_check_vel_continuous\"/>"
            "						</UniqueParam>"
            "						<UniqueParam default=\"not_check_vel_continuous_at_start\">"
            "							<Param name=\"check_vel_continuous_at_start\"/>"
            "							<Param name=\"not_check_vel_continuous_at_start\"/>"
            "						</UniqueParam>"
            "						<UniqueParam default=\"check_vel_following_error\">"
            "							<Param name=\"check_vel_following_error\"/>"
            "							<Param name=\"not_check_vel_following_error\"/>"
            "						</UniqueParam>"
            "					</GroupParam>"
            "				</UniqueParam>"
            "			</GroupParam>"
            "		</UniqueParam>"
			"	</GroupParam>"
			"</Command>");
	}


	auto createPlanRootRokaeXB4()->std::unique_ptr<aris::plan::PlanRoot>
	{
        std::unique_ptr<aris::plan::PlanRoot> plan_root(new aris::plan::PlanRoot);

		plan_root->planPool().add<aris::plan::Enable>();
		plan_root->planPool().add<aris::plan::Disable>();
		plan_root->planPool().add<aris::plan::Mode>();
		plan_root->planPool().add<aris::plan::Recover>();
		auto &rs = plan_root->planPool().add<aris::plan::Reset>();
        rs.command().findParam("pos")->setDefaultValue("{0.0,0.00,0.0}");

		plan_root->planPool().add<aris::plan::MoveL>();
        plan_root->planPool().add<aris::plan::GetXml>();
        plan_root->planPool().add<aris::plan::GetPartPq>();
        plan_root->planPool().add<aris::plan::SetXml>();
		plan_root->planPool().add<aris::plan::MoveJ>();
		plan_root->planPool().add<aris::plan::Show>();
		plan_root->planPool().add<kaanh::MoveJR>();
		plan_root->planPool().add<kaanh::ShowAll>();

/*
		plan_root->planPool().add<kaanh::moveC_3P>();
		plan_root->planPool().add<kaanh::moveC_CE>();
		plan_root->planPool().add<kaanh::moveC_RE>();
		plan_root->planPool().add<kaanh::moveL_Cos>();
		plan_root->planPool().add<kaanh::moveL_T>();
*/
		plan_root->planPool().add<kaanh::moveJ_Cos>();
        plan_root->planPool().add<kaanh::MoveCircle>();
        plan_root->planPool().add<kaanh::MoveCosFC>();
        plan_root->planPool().add<kaanh::MoveAndAcquire>();
        plan_root->planPool().add<kaanh::MoveJ3>();
		plan_root->planPool().add<kaanh::Sensor>();
        plan_root->planPool().add<kaanh::moveJ_T>();
        plan_root->planPool().add<kaanh::MoveJD>();
		plan_root->planPool().add<kaanh::ElbowTorqueControl>();

		return plan_root;
	}

}