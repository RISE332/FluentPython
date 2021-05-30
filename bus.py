from ortools.linear_solver import pywraplp
from ortools.sat.python import cp_model
import random


class LP_test:
    def __init__(self):
        # 车辆数
        self.bus = 44
        # 每辆车跑的班次数
        self.pre_bus_gotimes = [4 for _ in range(self.bus)]
        self.pre_bus_backtimes = [4 for _ in range(self.bus)]
        # from 6.15-22.00
        # from 6.15-22.30
        self.x = sum(self.pre_bus_gotimes + self.pre_bus_backtimes)
        # 先去后来和先来后去的分割线
        self.between = 28
        # 去的首版末班时间 6.15-22.00
        self.go_start_time = 375
        self.go_finish_time = 1320
        # 回的首版末班时间 6.15-22.30
        self.back_start_time = 375
        self.back_finish_time = 1350
        # 固定行驶时间
        self.trip_time = 70
        # 早高峰之前，早高峰，早高峰和晚高峰之间，晚高峰，晚高峰之后的班次
        self.go_before = 10
        self.go_morning = 27
        self.go_among = 84
        self.go_night = 27
        self.go_after = 28
        self.back_before = 10
        self.back_morning = 27
        self.back_among = 84
        self.back_night = 27
        self.back_after = 28
        # 高峰与平峰间隔最大最小值
        self.busy_min = 4
        self.busy_max = 5
        self.no_busy_min = 5
        self.no_busy_max = random.randint(5, 10)
        # 早高峰晚高峰时间
        self.morning_busy_time_start = 420
        self.morning_busy_time_finish = 540
        self.night_busy_time_start = 1020
        self.night_busy_time_finish = 1140

    def LP_solver(self):
        """
        规划求解器
        """
        solver = pywraplp.Solver('god cui', pywraplp.Solver.CBC_MIXED_INTEGER_PROGRAMMING)
        x = {}
        # 设置变量
        z = 0
        for i in range(self.bus):
            for j in range(self.pre_bus_gotimes[i] + self.pre_bus_backtimes[i]):
                if i < self.between:
                    if j % 2 == 0:
                        x[z] = solver.IntVar(self.go_start_time, self.go_finish_time, 'x[go,{},{}]'
                                             .format(i, j // 2))
                    else:
                        x[z] = solver.IntVar(self.back_start_time, self.back_finish_time, 'x[back,{},{}]'
                                             .format(i, j // 2))
                else:
                    if j % 2 == 0:
                        x[z] = solver.IntVar(self.back_start_time, self.back_finish_time, 'x[back,{},{}]'
                                             .format(i, j // 2))
                    else:
                        x[z] = solver.IntVar(self.go_start_time, self.go_finish_time, 'x[go,{},{}]'
                                             .format(i, j // 2))
                # print(x[z].name())
                z = z + 1

        constraint1 = []
        # 不等式约束，每辆车的相邻班次的间隔大于行驶时间
        for i in range(self.bus):
            for j in range(self.pre_bus_gotimes[i] + self.pre_bus_backtimes[i] - 1):
                constraint1.append(solver.RowConstraint(self.trip_time, solver.infinity(),
                                                        'constraint_go,{},{}'.format(i, j)))
                # print(constraint1[i*(self.pre_bus_gotimes[i]+self.pre_bus_backtimes[i]-1)+j].name())
                constraint1[i * (self.pre_bus_gotimes[i] + self.pre_bus_backtimes[i] - 1) + j] \
                    .SetCoefficient(x[i * (self.pre_bus_gotimes[i] + self.pre_bus_backtimes[i]) + j], -1)
                constraint1[i * (self.pre_bus_gotimes[i] + self.pre_bus_backtimes[i] - 1) + j] \
                    .SetCoefficient(x[i * (self.pre_bus_gotimes[i] + self.pre_bus_backtimes[i]) + j + 1], 1)

        # bus1 go-first = 6.15
        # index是某辆车的班次序号
        bus = 0
        index = 0
        constraint2 = solver.RowConstraint(self.go_start_time, self.go_start_time)
        constraint2.SetCoefficient(x[bus * (self.pre_bus_gotimes[0] + self.pre_bus_backtimes[0]) + index], 1)
        # bus44 go-last = 22.00
        bus = 43
        index = 7
        constraint3 = solver.RowConstraint(self.go_finish_time, self.go_finish_time)
        constraint3.SetCoefficient(x[bus * (self.pre_bus_gotimes[0] + self.pre_bus_backtimes[0]) + index], 1)
        # bus29 back-first = 6.15
        bus = 28
        index = 0
        constraint4 = solver.RowConstraint(self.back_start_time, self.back_start_time)
        constraint4.SetCoefficient(x[bus * (self.pre_bus_gotimes[0] + self.pre_bus_backtimes[0]) + index], 1)
        # bus28 back-last = 22.30
        bus = 27
        index = 7
        constraint5 = solver.RowConstraint(self.back_finish_time, self.back_finish_time)
        constraint5.SetCoefficient(x[bus * (self.pre_bus_gotimes[0] + self.pre_bus_backtimes[0]) + index], 1)

        # 班次的间隔约束
        constraint6 = []

        def Interval(bus1, bus2, index1, index2, time_min, time_max):
            """
            班次的间隔约束
            time in bus1<time in bus2
            input: 公交车，公交车的班次，最大最小间隔
            output: constraint6.add(constraint)
            """
            constraint6.append(solver.RowConstraint(time_min, time_max))
            constraint6[-1].SetCoefficient(x[bus1 * (self.pre_bus_gotimes[0] + self.pre_bus_backtimes[0])
                                             + index1], -1)
            constraint6[-1].SetCoefficient(x[bus2 * (self.pre_bus_gotimes[0] + self.pre_bus_backtimes[0])
                                             + index2], 1)

        # go_早高峰之前,班次的间隔约束
        # 去是从第一辆车开始的
        flag = 0
        for i in range(self.go_before):
            bus_before, index_before = LP_test().return_bus_index(i + 1, flag)
            bus_after, index_after = LP_test().return_bus_index(i + 2, flag)
            # print(bus_before, index_before)
            Interval(bus_before, bus_after, index_before, index_after, self.no_busy_min, self.no_busy_max)

        # go_早高峰,班次的间隔约束
        for i in range(self.go_before,
                       self.go_before + self.go_morning):
            bus_before, index_before = LP_test().return_bus_index(i + 1, flag)
            bus_after, index_after = LP_test().return_bus_index(i + 2, flag)
            Interval(bus_before, bus_after, index_before, index_after, self.busy_min, self.busy_max)

        # go_早高峰晚高峰之间,班次的间隔约束
        for i in range(self.go_before + self.go_morning,
                       self.go_before + self.go_morning + self.go_among):
            bus_before, index_before = LP_test().return_bus_index(i + 1, flag)
            bus_after, index_after = LP_test().return_bus_index(i + 2, flag)
            Interval(bus_before, bus_after, index_before, index_after, self.no_busy_min, self.no_busy_max)

        # go_晚高峰,班次的间隔约束
        for i in range(self.go_before + self.go_morning + self.go_among,
                       self.go_before + self.go_morning + self.go_among + self.go_night):
            bus_before, index_before = LP_test().return_bus_index(i + 1, flag)
            bus_after, index_after = LP_test().return_bus_index(i + 2, flag)
            Interval(bus_before, bus_after, index_before, index_after, self.busy_min, self.busy_max)

        # go_晚高峰之后,班次的间隔约束
        for i in range(self.go_before + self.go_morning + self.go_among + self.go_night,
                       self.go_before + self.go_morning + self.go_among + self.go_night + self.go_after - 1):
            bus_before, index_before = LP_test().return_bus_index(i + 1, flag)
            bus_after, index_after = LP_test().return_bus_index(i + 2, flag)
            Interval(bus_before, bus_after, index_before, index_after, self.no_busy_min, self.no_busy_max)

        # back_早高峰之前,班次的间隔约束
        # 去是从第29辆车开始的
        flag = 1
        for i in range(self.back_before):
            bus_before, index_before = LP_test().return_bus_index(i + 1, flag)
            bus_after, index_after = LP_test().return_bus_index(i + 2, flag)
            Interval(bus_before, bus_after, index_before, index_after,
                     self.no_busy_min, self.no_busy_max)

        # back_早高峰,班次的间隔约束
        for i in range(self.back_before,
                       self.back_before + self.back_morning):
            bus_before, index_before = LP_test().return_bus_index(i + 1, flag)
            bus_after, index_after = LP_test().return_bus_index(i + 2, flag)
            Interval(bus_before, bus_after, index_before, index_after, self.busy_min, self.busy_max)

        # back_早高峰晚高峰之间,班次的间隔约束
        for i in range(self.back_before + self.back_morning,
                       self.back_before + self.back_morning + self.back_among):
            bus_before, index_before = LP_test().return_bus_index(i + 1, flag)
            bus_after, index_after = LP_test().return_bus_index(i + 2, flag)
            Interval(bus_before, bus_after, index_before, index_after, self.no_busy_min, self.no_busy_max)

        # back_晚高峰,班次的间隔约束
        for i in range(self.back_before + self.back_morning + self.back_among,
                       self.back_before + self.back_morning + self.back_among + self.back_night):
            bus_before, index_before = LP_test().return_bus_index(i + 1, flag)
            bus_after, index_after = LP_test().return_bus_index(i + 2, flag)
            Interval(bus_before, bus_after, index_before, index_after, self.busy_min, self.busy_max)

        # back_晚高峰之后,班次的间隔约束
        for i in range(self.back_before + self.back_morning + self.back_among + self.back_night,
                       self.back_before + self.back_morning + self.back_among + self.back_night + self.back_after - 1):
            bus_before, index_before = LP_test().return_bus_index(i + 1, flag)
            bus_after, index_after = LP_test().return_bus_index(i + 2, flag)
            Interval(bus_before, bus_after, index_before, index_after, self.no_busy_min, self.no_busy_max)

        # 时间段约束,早高峰之前，早高峰，早高峰和晚高峰之间，晚高峰，晚高峰之后
        constraint7 = []

        # go
        flag = 0
        # 早高峰开始
        bus, index = LP_test().return_bus_index(self.go_before, flag)
        constraint7.append(solver.RowConstraint(-solver.infinity(), self.morning_busy_time_start))
        constraint7[-1].SetCoefficient(x[bus * (self.pre_bus_gotimes[0] + self.pre_bus_backtimes[0]) + index], 1)
        bus, index = LP_test().return_bus_index(self.go_before + 1, flag)
        constraint7.append(solver.RowConstraint(self.morning_busy_time_start, solver.infinity()))
        constraint7[-1].SetCoefficient(x[bus * (self.pre_bus_gotimes[0] + self.pre_bus_backtimes[0]) + index], 1)

        # 早高峰结束
        bus, index = LP_test().return_bus_index(self.go_before + self.go_morning, flag)
        constraint7.append(solver.RowConstraint(-solver.infinity(), self.morning_busy_time_finish))
        constraint7[-1].SetCoefficient(x[bus * (self.pre_bus_gotimes[0] + self.pre_bus_backtimes[0]) + index], 1)
        bus, index = LP_test().return_bus_index(self.go_before + self.go_morning + 1, flag)
        constraint7.append(solver.RowConstraint(self.morning_busy_time_finish, solver.infinity()))
        constraint7[-1].SetCoefficient(x[bus * (self.pre_bus_gotimes[0] + self.pre_bus_backtimes[0]) + index], 1)

        # 晚高峰开始
        bus, index = LP_test().return_bus_index(self.go_before + self.go_morning + self.go_among, flag)
        constraint7.append(solver.RowConstraint(-solver.infinity(), self.night_busy_time_start))
        constraint7[-1].SetCoefficient(x[bus * (self.pre_bus_gotimes[0] + self.pre_bus_backtimes[0]) + index], 1)
        bus, index = LP_test().return_bus_index(self.go_before + self.go_morning + self.go_among + 1, flag)
        constraint7.append(solver.RowConstraint(self.night_busy_time_start, solver.infinity()))
        constraint7[-1].SetCoefficient(x[bus * (self.pre_bus_gotimes[0] + self.pre_bus_backtimes[0]) + index], 1)

        # 晚高峰结束
        bus, index = LP_test().return_bus_index(self.go_before + self.go_morning + self.go_among + self.go_night, flag)
        constraint7.append(solver.RowConstraint(-solver.infinity(), self.night_busy_time_finish))
        constraint7[-1].SetCoefficient(x[bus * (self.pre_bus_gotimes[0] + self.pre_bus_backtimes[0]) + index], 1)
        bus, index = LP_test().return_bus_index(self.go_before + self.go_morning + self.go_among + self.go_night + 1,
                                                flag)
        constraint7.append(solver.RowConstraint(self.night_busy_time_finish, solver.infinity()))
        constraint7[-1].SetCoefficient(x[bus * (self.pre_bus_gotimes[0] + self.pre_bus_backtimes[0]) + index], 1)

        # back
        flag = 1
        # 早高峰开始
        bus, index = LP_test().return_bus_index(self.back_before, flag)
        constraint7.append(solver.RowConstraint(-solver.infinity(), self.morning_busy_time_start))
        constraint7[-1].SetCoefficient(x[bus * (self.pre_bus_gotimes[0] + self.pre_bus_backtimes[0]) + index], 1)
        bus, index = LP_test().return_bus_index(self.back_before + 1, flag)
        constraint7.append(solver.RowConstraint(self.morning_busy_time_start, solver.infinity()))
        constraint7[-1].SetCoefficient(x[bus * (self.pre_bus_gotimes[0] + self.pre_bus_backtimes[0]) + index], 1)

        # 早高峰结束
        bus, index = LP_test().return_bus_index(self.back_before + self.back_morning, flag)
        constraint7.append(solver.RowConstraint(-solver.infinity(), self.morning_busy_time_finish))
        constraint7[-1].SetCoefficient(x[bus * (self.pre_bus_gotimes[0] + self.pre_bus_backtimes[0]) + index], 1)
        bus, index = LP_test().return_bus_index(self.back_before + self.back_morning + 1, flag)
        constraint7.append(solver.RowConstraint(self.morning_busy_time_finish, solver.infinity()))
        constraint7[-1].SetCoefficient(x[bus * (self.pre_bus_gotimes[0] + self.pre_bus_backtimes[0]) + index], 1)

        # 晚高峰开始
        bus, index = LP_test().return_bus_index(self.back_before + self.back_morning + self.back_among, flag)
        constraint7.append(solver.RowConstraint(-solver.infinity(), self.night_busy_time_start))
        constraint7[-1].SetCoefficient(x[bus * (self.pre_bus_gotimes[0] + self.pre_bus_backtimes[0]) + index], 1)
        bus, index = LP_test().return_bus_index(self.back_before + self.back_morning + self.back_among + 1, flag)
        constraint7.append(solver.RowConstraint(self.night_busy_time_start, solver.infinity()))
        constraint7[-1].SetCoefficient(x[bus * (self.pre_bus_gotimes[0] + self.pre_bus_backtimes[0]) + index], 1)

        # 晚高峰结束
        bus, index = LP_test().return_bus_index(self.back_before
                                                + self.back_morning + self.back_among + self.back_night, flag)
        constraint7.append(solver.RowConstraint(-solver.infinity(), self.night_busy_time_finish))
        constraint7[-1].SetCoefficient(x[bus * (self.pre_bus_gotimes[0] + self.pre_bus_backtimes[0]) + index], 1)
        bus, index = LP_test().return_bus_index(self.back_before
                                                + self.back_morning + self.back_among + self.back_night + 1, flag)
        constraint7.append(solver.RowConstraint(self.night_busy_time_finish, solver.infinity()))
        constraint7[-1].SetCoefficient(x[bus * (self.pre_bus_gotimes[0] + self.pre_bus_backtimes[0]) + index], 1)

        status = solver.Solve()
        print('has more solution? {}'.format(solver.NextSolution()))
        if status == pywraplp.Solver.OPTIMAL:
            print('Objective value =', solver.Objective().Value())
            for i in range(self.bus):
                x_print = []
                if i < self.between:
                    x_print.append('   ')
                for j in range(self.pre_bus_gotimes[i] + self.pre_bus_backtimes[i]):
                    # x_print.append(x[i*(self.pre_bus_gotimes[i] + self.pre_bus_backtimes[i])+j].solution_value())
                    x_print.append('{}.{}'
                                   .format(int(x[i * (self.pre_bus_gotimes[i] + self.pre_bus_backtimes[i]) + j]
                                               .solution_value() // 60),
                                           int(x[i * (self.pre_bus_gotimes[i] + self.pre_bus_backtimes[i]) + j]
                                               .solution_value() % 60)))
                print("bus{}:{}".format(i + 1, x_print))
            print()
            print('Problem solved in %f milliseconds' % solver.wall_time())
            print('Problem solved in %d iterations' % solver.iterations())
            print('Problem solved in %d branch-and-bound nodes' % solver.nodes())
        else:
            print('The problem does not have an optimal solution.')

    def LP_cp_model(self):
        solver = cp_model.CpModel()
        x = {}
        x_list = []
        obj = 0
        # 设置变量
        z = 0
        for i in range(self.bus):
            for j in range(self.pre_bus_gotimes[i] + self.pre_bus_backtimes[i]):
                if i < self.between:
                    if j % 2 == 0:
                        x[z] = solver.NewIntVar(self.go_start_time, self.go_finish_time, 'x[go,{},{}]'
                                                .format(i, j // 2))
                    else:
                        x[z] = solver.NewIntVar(self.back_start_time, self.back_finish_time, 'x[back,{},{}]'
                                                .format(i, j // 2))
                else:
                    if j % 2 == 0:
                        x[z] = solver.NewIntVar(self.back_start_time, self.back_finish_time, 'x[back,{},{}]'
                                                .format(i, j // 2))
                    else:
                        x[z] = solver.NewIntVar(self.go_start_time, self.go_finish_time, 'x[go,{},{}]'
                                                .format(i, j // 2))
                # print(x[z])
                x_list.append(x[z])
                z = z + 1

        # 不等式约束，每辆车的相邻班次的间隔大于行驶时间
        for i in range(self.bus):
            for j in range(self.pre_bus_gotimes[i] + self.pre_bus_backtimes[i] - 1):
                solver.Add(x[i * (self.pre_bus_gotimes[i] + self.pre_bus_backtimes[i]) + j + 1] -
                           x[i * (self.pre_bus_gotimes[i] + self.pre_bus_backtimes[i]) + j] >= self.trip_time)

        # bus1 go-first = 6.15
        # index是某辆车的班次序号
        bus = 0
        index = 0
        solver.Add(x[bus * (self.pre_bus_gotimes[0] + self.pre_bus_backtimes[0]) + index] == self.go_start_time)
        # bus44 go-last = 22.00
        bus = 43
        index = 7
        solver.Add(x[bus * (self.pre_bus_gotimes[0] + self.pre_bus_backtimes[0]) + index] == self.go_finish_time)
        # bus29 back-first = 6.15
        bus = 28
        index = 0
        solver.Add(x[bus * (self.pre_bus_gotimes[0] + self.pre_bus_backtimes[0]) + index] == self.back_start_time)
        # bus28 back-last = 22.30
        bus = 27
        index = 7
        solver.Add(x[bus * (self.pre_bus_gotimes[0] + self.pre_bus_backtimes[0]) + index] == self.back_finish_time)

        def Interval(bus1, bus2, index1, index2, time_min, time_max):
            """
            班次的间隔约束
            time in bus1<time in bus2
            input: 公交车，公交车的班次，最大最小间隔
            output: solver.Add(constraint)
            """
            solver.Add(x[bus2 * (self.pre_bus_gotimes[0] + self.pre_bus_backtimes[0]) + index2] -
                       x[bus1 * (self.pre_bus_gotimes[0] + self.pre_bus_backtimes[0]) + index1] <= time_max)
            solver.Add(x[bus2 * (self.pre_bus_gotimes[0] + self.pre_bus_backtimes[0]) + index2] -
                       x[bus1 * (self.pre_bus_gotimes[0] + self.pre_bus_backtimes[0]) + index1] >= time_min)

        # go_早高峰之前,班次的间隔约束
        # 去是从第一辆车开始的
        flag = 0
        for i in range(self.go_before):
            bus_before, index_before = LP_test().return_bus_index(i + 1, flag)
            bus_after, index_after = LP_test().return_bus_index(i + 2, flag)
            # print(bus_before, index_before)
            Interval(bus_before, bus_after, index_before, index_after, self.no_busy_min, self.no_busy_max)

        # go_早高峰,班次的间隔约束
        for i in range(self.go_before,
                       self.go_before + self.go_morning):
            bus_before, index_before = LP_test().return_bus_index(i + 1, flag)
            bus_after, index_after = LP_test().return_bus_index(i + 2, flag)
            Interval(bus_before, bus_after, index_before, index_after, self.busy_min, self.busy_max)

        # go_早高峰晚高峰之间,班次的间隔约束
        for i in range(self.go_before + self.go_morning,
                       self.go_before + self.go_morning + self.go_among):
            bus_before, index_before = LP_test().return_bus_index(i + 1, flag)
            bus_after, index_after = LP_test().return_bus_index(i + 2, flag)
            Interval(bus_before, bus_after, index_before, index_after, self.no_busy_min, self.no_busy_max)

        # go_晚高峰,班次的间隔约束
        for i in range(self.go_before + self.go_morning + self.go_among,
                       self.go_before + self.go_morning + self.go_among + self.go_night):
            bus_before, index_before = LP_test().return_bus_index(i + 1, flag)
            bus_after, index_after = LP_test().return_bus_index(i + 2, flag)
            Interval(bus_before, bus_after, index_before, index_after, self.busy_min, self.busy_max)

        # go_晚高峰之后,班次的间隔约束
        for i in range(self.go_before + self.go_morning + self.go_among + self.go_night,
                       self.go_before + self.go_morning + self.go_among + self.go_night + self.go_after - 1):
            bus_before, index_before = LP_test().return_bus_index(i + 1, flag)
            bus_after, index_after = LP_test().return_bus_index(i + 2, flag)
            Interval(bus_before, bus_after, index_before, index_after, self.no_busy_min, self.no_busy_max)

        # back_早高峰之前,班次的间隔约束
        # 去是从第29辆车开始的
        flag = 1
        for i in range(self.back_before):
            bus_before, index_before = LP_test().return_bus_index(i + 1, flag)
            bus_after, index_after = LP_test().return_bus_index(i + 2, flag)
            Interval(bus_before, bus_after, index_before, index_after,
                     self.no_busy_min, self.no_busy_max)

        # back_早高峰,班次的间隔约束
        for i in range(self.back_before,
                       self.back_before + self.back_morning):
            bus_before, index_before = LP_test().return_bus_index(i + 1, flag)
            bus_after, index_after = LP_test().return_bus_index(i + 2, flag)
            Interval(bus_before, bus_after, index_before, index_after, self.busy_min, self.busy_max)

        # back_早高峰晚高峰之间,班次的间隔约束
        for i in range(self.back_before + self.back_morning,
                       self.back_before + self.back_morning + self.back_among):
            bus_before, index_before = LP_test().return_bus_index(i + 1, flag)
            bus_after, index_after = LP_test().return_bus_index(i + 2, flag)
            Interval(bus_before, bus_after, index_before, index_after, self.no_busy_min, self.no_busy_max)

        # back_晚高峰,班次的间隔约束
        for i in range(self.back_before + self.back_morning + self.back_among,
                       self.back_before + self.back_morning + self.back_among + self.back_night):
            bus_before, index_before = LP_test().return_bus_index(i + 1, flag)
            bus_after, index_after = LP_test().return_bus_index(i + 2, flag)
            Interval(bus_before, bus_after, index_before, index_after, self.busy_min, self.busy_max)

        # back_晚高峰之后,班次的间隔约束
        for i in range(self.back_before + self.back_morning + self.back_among + self.back_night,
                       self.back_before + self.back_morning + self.back_among + self.back_night + self.back_after - 1):
            bus_before, index_before = LP_test().return_bus_index(i + 1, flag)
            bus_after, index_after = LP_test().return_bus_index(i + 2, flag)
            Interval(bus_before, bus_after, index_before, index_after, self.no_busy_min, self.no_busy_max)

        # 时间段约束,早高峰之前，早高峰，早高峰和晚高峰之间，晚高峰，晚高峰之后
        # go
        flag = 0
        # 早高峰开始
        bus, index = LP_test().return_bus_index(self.go_before, flag)
        solver.Add(
            x[bus * (self.pre_bus_gotimes[0] + self.pre_bus_backtimes[0]) + index] <= self.morning_busy_time_start)
        bus, index = LP_test().return_bus_index(self.go_before + 1, flag)
        solver.Add(
            x[bus * (self.pre_bus_gotimes[0] + self.pre_bus_backtimes[0]) + index] >= self.morning_busy_time_start)

        # 早高峰结束
        bus, index = LP_test().return_bus_index(self.go_before + self.go_morning, flag)
        solver.Add(
            x[bus * (self.pre_bus_gotimes[0] + self.pre_bus_backtimes[0]) + index] <= self.morning_busy_time_finish)
        bus, index = LP_test().return_bus_index(self.go_before + self.go_morning + 1, flag)
        solver.Add(
            x[bus * (self.pre_bus_gotimes[0] + self.pre_bus_backtimes[0]) + index] >= self.morning_busy_time_finish)

        # 晚高峰开始
        bus, index = LP_test().return_bus_index(self.go_before + self.go_morning + self.go_among, flag)
        solver.Add(x[bus * (self.pre_bus_gotimes[0] + self.pre_bus_backtimes[0]) + index] <= self.night_busy_time_start)
        bus, index = LP_test().return_bus_index(self.go_before + self.go_morning + self.go_among + 1, flag)
        solver.Add(x[bus * (self.pre_bus_gotimes[0] + self.pre_bus_backtimes[0]) + index] >= self.night_busy_time_start)

        # 晚高峰结束
        bus, index = LP_test().return_bus_index(self.go_before + self.go_morning + self.go_among + self.go_night, flag)
        solver.Add(
            x[bus * (self.pre_bus_gotimes[0] + self.pre_bus_backtimes[0]) + index] <= self.night_busy_time_finish)
        bus, index = LP_test().return_bus_index(self.go_before + self.go_morning + self.go_among + self.go_night + 1,
                                                flag)
        solver.Add(
            x[bus * (self.pre_bus_gotimes[0] + self.pre_bus_backtimes[0]) + index] >= self.night_busy_time_finish)

        # back
        flag = 1
        # 早高峰开始
        bus, index = LP_test().return_bus_index(self.back_before, flag)
        solver.Add(
            x[bus * (self.pre_bus_gotimes[0] + self.pre_bus_backtimes[0]) + index] <= self.morning_busy_time_start)
        bus, index = LP_test().return_bus_index(self.back_before + 1, flag)
        solver.Add(
            x[bus * (self.pre_bus_gotimes[0] + self.pre_bus_backtimes[0]) + index] >= self.morning_busy_time_start)

        # 早高峰结束
        bus, index = LP_test().return_bus_index(self.back_before + self.back_morning, flag)
        solver.Add(
            x[bus * (self.pre_bus_gotimes[0] + self.pre_bus_backtimes[0]) + index] <= self.morning_busy_time_finish)
        bus, index = LP_test().return_bus_index(self.back_before + self.back_morning + 1, flag)
        solver.Add(
            x[bus * (self.pre_bus_gotimes[0] + self.pre_bus_backtimes[0]) + index] >= self.morning_busy_time_finish)

        # 晚高峰开始
        bus, index = LP_test().return_bus_index(self.back_before + self.back_morning + self.back_among, flag)
        solver.Add(x[bus * (self.pre_bus_gotimes[0] + self.pre_bus_backtimes[0]) + index] <= self.night_busy_time_start)
        bus, index = LP_test().return_bus_index(self.back_before + self.back_morning + self.back_among + 1, flag)
        solver.Add(x[bus * (self.pre_bus_gotimes[0] + self.pre_bus_backtimes[0]) + index] >= self.night_busy_time_start)

        # 晚高峰结束
        bus, index = LP_test().return_bus_index(self.back_before +
                                                self.back_morning + self.back_among + self.back_night, flag)
        solver.Add(
            x[bus * (self.pre_bus_gotimes[0] + self.pre_bus_backtimes[0]) + index] <= self.night_busy_time_finish)
        bus, index = LP_test().return_bus_index(self.back_before +
                                                self.back_morning + self.back_among + self.back_night + 1, flag)
        solver.Add(
            x[bus * (self.pre_bus_gotimes[0] + self.pre_bus_backtimes[0]) + index] >= self.night_busy_time_finish)

        # 求可行解

        # solver_1 = cp_model.CpSolver()
        # solution_printer = LP_test.VarArraySolutionPrinter(x_list)
        # status = solver_1.SearchForAllSolutions(solver, solution_printer)
        # print('Status = %s' % solver.StatusName(status))
        # print('Number of solutions found: %i' % solution_printer.solution_count())

        # 求部分可行解
        solver_1 = cp_model.CpSolver()
        solution_printer = LP_test.VarArraySolutionPrinterWithLimit(x_list, 2)
        status = solver_1.SearchForAllSolutions(solver, solution_printer)
        print('Status = %s' % status)
        print('Number of solutions found: %i' % solution_printer.solution_count())

    def return_bus_index(self, num, flag):
        """
        班次转化为车辆和车辆的班次
        num -> [bus, index]
        inout: 班次
        flag=0->go
        flag=1->back
        output: [bus, index] -> int
        """
        if flag == 0:
            bus = num % self.bus - 1
            if bus == -1:
                bus = bus + self.bus
            index = 2 * (num // self.bus)
            if bus >= self.between:
                index = index + 1
            if bus == self.bus - 1:
                index = 2 * (num // self.bus - 1) + 1
        else:
            num = num + 28
            bus = num % self.bus - 1
            if bus == -1:
                bus = bus + self.bus
            index = 2 * (num // self.bus)
            if bus < self.between:
                index = index - 1
            if bus == self.bus - 1:
                index = 2 * (num // self.bus - 1)
        return bus, index

    class VarArraySolutionPrinterWithLimit(cp_model.CpSolverSolutionCallback):
        """Print intermediate solutions."""

        def __init__(self, variables, limit):
            cp_model.CpSolverSolutionCallback.__init__(self)
            self.__variables = variables
            self.__solution_count = 0
            self.__solution_limit = limit

        def on_solution_callback(self):
            self.__solution_count += 1
            for v in self.__variables:
                print('%s=%i' % (v, self.Value(v)), end=' ')
            print()
            if self.__solution_count >= self.__solution_limit:
                print('Stop search after %i solutions' % self.__solution_limit)
                self.StopSearch()

        def solution_count(self):
            return self.__solution_count

    class VarArraySolutionPrinter(cp_model.CpSolverSolutionCallback):
        """Print intermediate solutions."""

        def __init__(self, variables):
            cp_model.CpSolverSolutionCallback.__init__(self)
            self.__variables = variables
            self.__solution_count = 0

        def on_solution_callback(self):
            self.__solution_count += 1
            for v in self.__variables:
                print('%s=%i' % (v, self.Value(v)), end=' ')
            print()

        def solution_count(self):
            return self.__solution_count


LP_test().LP_cp_model()
# LP_test().LP_solver()


