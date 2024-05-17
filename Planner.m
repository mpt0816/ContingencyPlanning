classdef Planner < handle
    properties
        num_of_share_;
        num_of_nominal_;
        num_of_contingency_;
        num_of_knots_;
        config_;
        init_state_;
        weights_;
        limits_;
        cut_in_;
        
        p_nominal_;
        p_contingency_;
    end
    
    methods
        function obj = Planner()
            disp("Init Planner.");
        end
        
        function speed = Plan(obj, config)
            obj.config_ = config;
            obj.num_of_share_ = max(1, floor(obj.config_.time.share / obj.config_.time.delta));
            time_contingency = obj.config_.time.horizon - obj.config_.time.share;
            obj.num_of_nominal_ = max(1, floor(time_contingency / obj.config_.time.delta));
            obj.num_of_contingency_ = max(1, floor(time_contingency / obj.config_.time.delta));
            obj.num_of_knots_ = obj.num_of_share_ + obj.num_of_nominal_ + obj.num_of_contingency_;
            
            obj.init_state_ = config.init_state;
            obj.weights_ = config.weights;
            obj.limits_ = config.limits;
            obj.p_nominal_ = config.p_nominal;
            obj.p_contingency_ = config.p_contingency;
            
            obj.cut_in_ = config.cut_in;
            
            problem = obj.FormulateQPProblem();
            qp_results = obj.SolveQPProblem(problem);
            
            if qp_results.info.status_val < 0 || (~(qp_results.info.status_val == 1) && ~(qp_results.info.status_val == 2))
                disp('Osqp Failed');
                return;
            end
            
            disp("Planned success.");
            
            speed = obj.DataTransformer(qp_results.x);
            
        end
        
        function obstacle = CutInObstacleST(obj)
            t = [];
            s = [];
            v = [];
            num = floor((obj.config_.time.horizon - obj.cut_in_.time) / obj.config_.time.delta);
            for i = 0 : 1 : num
                t_tem = obj.cut_in_.time + i * obj.config_.time.delta;
                s_tem = obj.cut_in_.s + i * obj.config_.time.delta * obj.cut_in_.speed;
                v_tem = obj.cut_in_.speed;
                t = [t, t_tem];
                s = [s, s_tem];
                v = [v, v_tem];
            end
            obstacle.t = t;
            obstacle.s = s;
            obstacle.v = v;
        end
        
        function speed = DataTransformer(obj, results)
            share.t = [];
            share.s = [];
            share.v = [];
            share.a = [];
            share.j = [];
            
            for i = 0 : 1 : obj.num_of_share_ - 1
                t = i * obj.config_.time.delta;
                share.t = [share.t, t];
                share.s = [share.s, results(4 * i + 1)];
                share.v = [share.v, results(4 * i + 2)];
                share.a = [share.a, results(4 * i + 3)];
                share.j = [share.j, results(4 * i + 4)];
            end
            
            nominal.t = [];
            nominal.s = [];
            nominal.v = [];
            nominal.a = [];
            nominal.j = [];
            for i = obj.num_of_share_ - 1 : 1 : obj.num_of_share_ + obj.num_of_nominal_ - 1
                t = i * obj.config_.time.delta;
                nominal.t = [nominal.t, t];
                nominal.s = [nominal.s, results(4 * i + 1)];
                nominal.v = [nominal.v, results(4 * i + 2)];
                nominal.a = [nominal.a, results(4 * i + 3)];
                nominal.j = [nominal.j, results(4 * i + 4)];
            end
            
            contingency.t = [share.t(end)];
            contingency.s = [share.s(end)];
            contingency.v = [share.v(end)];
            contingency.a = [share.a(end)];
            contingency.j = [share.j(end)];
            for i = obj.num_of_share_ +  obj.num_of_nominal_ : 1 : obj.num_of_knots_ - 1
                t = (i - obj.num_of_share_ -  obj.num_of_nominal_ + 1) * obj.config_.time.delta + share.t(end);
                contingency.t = [contingency.t, t];
                contingency.s = [contingency.s, results(4 * i + 1)];
                contingency.v = [contingency.v, results(4 * i + 2)];
                contingency.a = [contingency.a, results(4 * i + 3)];
                contingency.j = [contingency.j, results(4 * i + 4)];
            end
            
            speed.share = share;
            speed.nominal = nominal;
            speed.contingency = contingency;
        end
        
        function problem = FormulateQPProblem(obj)
            kernel = obj.CalcaulateKernel();
            problem.H = kernel.H;
            problem.G = kernel.G;
            constraint =obj. CalculateConstraint();
            problem.A = constraint.A;
            problem.lower = constraint.lower;
            problem.upper = constraint.upper;
        end
        
        function result = SolveQPProblem(obj, problem)
            P = sparse(problem.H);
            q = problem.G;
            A = sparse(problem.A);
            l = problem.lower;
            u = problem.upper;
            
            solver = osqp;
            settings = obj.OsqpSettings(solver);
            solver.setup(P, q, A, l, u, settings);
            result = solver.solve();
        end
        function settings = OsqpSettings(obj, solver)
            settings = solver.default_settings();
            settings.max_iter = 5000;
            settings.polish = true;
            settings.verbose = false;
            settings.scaled_termination = false;
            settings.warm_start = false;
        end
        
        function kernel = CalcaulateKernel(obj)
            %% state: s, v, a, control: j
            kernel_dim = 4 * obj.num_of_knots_;
            kernel.H = zeros(kernel_dim, kernel_dim);
            kernel.G = zeros(kernel_dim, 1);
            
            for i = 0 : 1 : obj.num_of_share_ - 1
                kernel.H(4 * i + 2, 4 * i + 2) = obj.weights_.seepd;
                kernel.H(4 * i + 4, 4 * i + 4) = obj.weights_.jerk;
            end
            
            for i = obj.num_of_share_ : 1 : obj.num_of_share_ + obj.num_of_nominal_ - 1
                kernel.H(4 * i + 2, 4 * i + 2) = obj.p_nominal_ * obj.weights_.seepd;
                kernel.H(4 * i + 4, 4 * i + 4) = obj.p_nominal_ * obj.weights_.jerk;
            end
            
            for i = obj.num_of_share_ + obj.num_of_nominal_ : 1 : obj.num_of_share_ + obj.num_of_nominal_ + obj.num_of_contingency_ - 1
                kernel.H(4 * i + 2, 4 * i + 2) = obj.p_contingency_ * obj.weights_.seepd;
                kernel.H(4 * i + 4, 4 * i + 4) = obj.p_contingency_ * obj.weights_.jerk;
            end
            
            target_speed = obj.init_state_.v;
            
            for i = 0 : 1 : obj.num_of_share_ - 1
                kernel.G(4 * i + 2, 1) = -obj.weights_.seepd * target_speed;
            end
            
            for i = obj.num_of_share_ : 1 : obj.num_of_share_ + obj.num_of_nominal_ - 1
                kernel.G(4 * i + 2, 1) = -obj.p_nominal_ * obj.weights_.seepd * target_speed;
            end
            
            for i = obj.num_of_share_ + obj.num_of_nominal_ : 1 : obj.num_of_share_ + obj.num_of_nominal_ + obj.num_of_contingency_ - 1
                kernel.G(4 * i + 2, 1) = -obj.p_contingency_ * obj.weights_.seepd * target_speed;
            end
            
        end
        
        function constraint = CalculateConstraint(obj)
            dt = obj.config_.time.delta;
            dt2 = dt * dt;
            dt3 = dt * dt2;
            
            kernel_dim = 4 * obj.num_of_knots_;
            num_of_constriant = 9 * obj.num_of_knots_ - 2;
            if obj.p_contingency_ > 1e-5
                cut_in_start_index = obj.CutInStartIndex();
                num_of_contingency_collison = obj.num_of_knots_ - cut_in_start_index + 1;
                num_of_constriant = num_of_constriant + num_of_contingency_collison;
            end
            A = zeros(num_of_constriant, kernel_dim);
            lower = zeros(num_of_constriant, 1);
            upper = zeros(num_of_constriant, 1);
            
            rows = 1;
            %% 起点约束, 4个
            A(rows : rows + 3, 1 : 4) = eye(4);
            lower(rows : rows + 3, 1) = [0; obj.init_state_.v; obj.init_state_.a; 0];
            upper(rows : rows + 3, 1) = [0; obj.init_state_.v; obj.init_state_.a; 0];
            rows = rows + 4;
            
            %% 运动学约束, share + nominal + contingency
            %% share : 3 * (num_of_share - 1)
            for i = 0 : 1 : obj.num_of_share_ - 2
                %% acc, jerk
                A(rows, 4 * i + 3 : 4 * i + 4) = [1, dt];
                A(rows, 4 * (i + 1) + 3) = -1;
                lower(rows, 1) = 0;
                upper(rows, 1) = 0;
                rows = rows + 1;
                
                %% v, acc, jerk
                A(rows, 4 * i + 2 : 4 * i + 4) = [1, dt, 0.5 * dt2];
                A(rows, 4 * (i + 1) + 2) = -1;
                lower(rows, 1) = 0;
                upper(rows, 1) = 0;
                rows = rows + 1;
                
                %% s, v, acc, jerk
                A(rows, 4 * i + 1 : 4 * i + 4) = [1, dt, 0.5 * dt2, 1/6 * dt3];
                A(rows, 4 * (i + 1) + 1) = -1;
                lower(rows, 1) = 0;
                upper(rows, 1) = 0;
                rows = rows + 1;
            end
            
            %% nominal : 3 * (num_of_nominal)， 因为需要包括share端的最后一个点
            for i = obj.num_of_share_ - 1 : 1 : obj.num_of_share_ + obj.num_of_nominal_ - 2
                %% acc, jerk
                A(rows, 4 * i + 3 : 4 * i + 4) = [1, dt];
                A(rows, 4 * (i + 1) + 3) = -1;
                lower(rows, 1) = 0;
                upper(rows, 1) = 0;
                rows = rows + 1;
                
                %% v, acc, jerk
                A(rows, 4 * i + 2 : 4 * i + 4) = [1, dt, 0.5 * dt2];
                A(rows, 4 * (i + 1) + 2) = -1;
                lower(rows, 1) = 0;
                upper(rows, 1) = 0;
                rows = rows + 1;
                
                %% s, v, acc, jerk
                A(rows, 4 * i + 1 : 4 * i + 4) = [1, dt, 0.5 * dt2, 1/6 * dt3];
                A(rows, 4 * (i + 1) + 1) = -1;
                lower(rows, 1) = 0;
                upper(rows, 1) = 0;
                rows = rows + 1;
            end
            
            %% contingency : 3 * (contingency)， 因为需要包括share端的最后一个点
            %% 先约束share段的最后一点个和contingency段的第一个点： 3
            %% acc, jerk
            A(rows, 4 * (obj.num_of_share_ - 1) + 3 : 4 * (obj.num_of_share_ - 1) + 4) = [1, dt];
            A(rows, 4 * (obj.num_of_share_ + obj.num_of_nominal_) + 3) = -1;
            lower(rows, 1) = 0;
            upper(rows, 1) = 0;
            rows = rows + 1;
            
            %% v, acc, jerk
            A(rows, 4 * (obj.num_of_share_ - 1) + 2 : 4 * (obj.num_of_share_ - 1) + 4) = [1, dt, 0.5 * dt2];
            A(rows, 4 * (obj.num_of_share_ + obj.num_of_nominal_) + 2) = -1;
            lower(rows, 1) = 0;
            upper(rows, 1) = 0;
            rows = rows + 1;
            
            %% s, v, acc, jerk
            A(rows, 4 * (obj.num_of_share_ - 1) + 1 : 4 * (obj.num_of_share_ - 1) + 4) = [1, dt, 0.5 * dt2, 1/6 * dt3];
            A(rows, 4 * (obj.num_of_share_ + obj.num_of_nominal_) + 1) = -1;
            lower(rows, 1) = 0;
            upper(rows, 1) = 0;
            rows = rows + 1;
            
            %% contingency : 3 * (contingency)， 因为需要包括share端的最后一个点
            %% 再约束contingency段： 3 * (contingency - 1)
            for i = obj.num_of_share_ + obj.num_of_nominal_ : 1 : obj.num_of_share_ + obj.num_of_nominal_ + obj.num_of_contingency_ - 2
                %% acc, jerk
                A(rows, 4 * i + 3 : 4 * i + 4) = [1, dt];
                A(rows, 4 * (i + 1) + 3) = -1;
                lower(rows, 1) = 0;
                upper(rows, 1) = 0;
                rows = rows + 1;
                
                %% v, acc, jerk
                A(rows, 4 * i + 2 : 4 * i + 4) = [1, dt, 0.5 * dt2];
                A(rows, 4 * (i + 1) + 2) = -1;
                lower(rows, 1) = 0;
                upper(rows, 1) = 0;
                rows = rows + 1;
                
                %% s, v, acc, jerk
                A(rows, 4 * i + 1 : 4 * i + 4) = [1, dt, 0.5 * dt2, 1/6 * dt3];
                A(rows, 4 * (i + 1) + 1) = -1;
                lower(rows, 1) = 0;
                upper(rows, 1) = 0;
                rows = rows + 1;
            end
            
            %% 只能前进，不能倒车, s的约束, n-1 个
            %% share段: share - 1
            for i = 0 : 1 : obj.num_of_share_ - 2
                A(rows, 4 * i + 1) = -1;
                A(rows, 4 * (i + 1) + 1) = 1;
                lower(rows, 1) = 0;
                upper(rows, 1) = 1e10;
                rows = rows + 1;
            end
            %% nominal段：nominal
            for i = obj.num_of_share_ : 1 : obj.num_of_share_ + obj.num_of_nominal_ - 2
                A(rows, 4 * i + 1) = -1;
                A(rows, 4 * (i + 1) + 1) = 1;
                lower(rows, 1) = 0;
                upper(rows, 1) = 1e10;
                rows = rows + 1;
            end
            
            %% contingency段：contingency
            A(rows, 4 * (obj.num_of_share_ - 1) + 1) = -1;
            A(rows, 4 * (obj.num_of_share_ + obj.num_of_nominal_) + 1) = 1;
            lower(rows, 1) = 0;
            upper(rows, 1) = 1e10;
            rows = rows + 1;
            for i = obj.num_of_share_ + obj.num_of_nominal_ : 1 : obj.num_of_share_ + obj.num_of_nominal_ + obj.num_of_contingency_ - 2
                A(rows, 4 * i + 1) = -1;
                A(rows, 4 * (i + 1) + 1) = 1;
                lower(rows, 1) = 0;
                upper(rows, 1) = 1e10;
                rows = rows + 1;
            end

            %% 速度,add, jerk, 3n个
            for i = 0 : 1 : obj.num_of_knots_ - 1
                A(rows : rows + 2, 4 * i + 2 : 4 * i + 4) = eye(3);
                lower(rows : rows + 2, 1) = [0; -obj.limits_.a; -obj.limits_.jerk];
                upper(rows : rows + 2, 1) = [obj.limits_.speed; obj.limits_.a; obj.limits_.jerk];
                rows = rows + 3;
            end

            %% contingency，cut in 约束
            %% 只对s约束
            if obj.p_contingency_ > 1e-5
                cut_in_start_index = obj.CutInStartIndex();
                for i = cut_in_start_index : 1 : obj.num_of_knots_ - 1
                    s = obj.CutInS(i);
                    A(rows, 4 * i + 1) = 1;
                    lower(rows, 1) = 0;
                    upper(rows, 1) = s;
                    rows = rows + 1;
                end
            end
            
            constraint.A = A;
            constraint.lower = lower;
            constraint.upper = upper;
        end
        
        function index = CutInStartIndex(obj)
            index = floor(obj.cut_in_.time / obj.config_.time.delta) + obj.num_of_nominal_;
        end
        
        function s = CutInS(obj, index)
            s = obj.cut_in_.s + ((index - obj.num_of_nominal_)* obj.config_.time.delta - obj.cut_in_.time) * obj.cut_in_.speed;
        end
        
        
        
        
    end
end
