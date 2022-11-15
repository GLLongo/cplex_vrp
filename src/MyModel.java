import ilog.concert.*;
import ilog.cplex.*;

import java.util.Random;

public class MyModel{
    private IloCplex cplex = new IloCplex();

    //variables
    private IloIntVar [][][] x; //x[h][i][j] edge pkg i-> pkg j is used by car h
    private IloIntVar [][][] y; //y[h][i][j] car h leaves depot i to pkg j
    private IloIntVar [][] z; //z[i][j] car j serves package i last - i.e edge from pkg i to dest j is used
    private IloIntVar [][] beta; //beta[i][j]  car i uses depot j - i.e edge from car i start to depot j is used

    private IloIntVar [][] t; //order variable for MTZ constraint t[h][j]>=t[h][j] - B(1-x[h][i][j])


    //constants
    //TODO - should these be ints
    private double [][] c; //c[i][j] cost (distance) of edge from pkg i to pkg j
    private double [][] c1;//c1[i][j] cost of edge from depot i to pkg j
    private double [][] c2; //c2[i][j] cost of edge from pkg i to car destination j
    private double [][] c3;//c3[i][j] cost of edge from car i start to depot j
    private int [] n; //n[h] maximum number of packages car h can deliver

    private int numPkgs;
    private int numCars;
    private int numDepots;

    public MyModel() throws IloException {
        //TODO
        //  - this should init my model as written in my notes with all variables and constraints and the objective

        this.parseConstants();


        //***********************************//
        //using little test instance
        Random rand = new Random(2000);

        this.numPkgs = 4;
        this.numCars = 3;
        this.numDepots = 4;

        this.c = new double[this.numPkgs][this.numPkgs];
        this.c1 = new double[this.numDepots][this.numPkgs];
        this.c2 = new double[this.numPkgs][this.numCars];
        this.c3 = new double[this.numCars][this.numDepots];
        this.n = new int[this.numCars];


        // rand.nextInt(high-low) + low;
        // populate c[][] with random values between 1 and 15
        for(int i = 0; i < numPkgs;i++){
            for(int j = 0; j < numPkgs;j++){
                c[i][j] = rand.nextInt(14) + 1;
            }
        }

        // populate c1[][] with random values between 1 and 15
        for(int i = 0; i < numDepots;i++){
            for(int j = 0; j < numPkgs;j++){
                c1[i][j] = rand.nextInt(14) + 1;
            }
        }

        // populate c2[][] with random values between 1 and 15
        for(int i = 0; i < numPkgs;i++){
            for(int j = 0; j < numCars;j++){
                c2[i][j] = rand.nextInt(14) + 1;
            }
        }

        // populate c3[][] with random values between 1 and 15
        for(int i = 0; i < numCars;i++){
            for(int j = 0; j < numDepots;j++){
                c3[i][j] = rand.nextInt(14) + 1;
            }
        }

        // populate n[] with random values between 1 and 4
        for(int h = 0; h < numCars;h++){
            n[h] =rand.nextInt(3) + 1;
        }

        //*******************************//




        //populate

        this.addVariables();
        this.addObjective();
        this.addConstraint1();
        this.addConstraint2();
        this.addConstraint3();
        this.addConstraint4();
        this.addConstraint5();
        this.addConstraint8();


    }


    private void parseConstants(){
        //TODO
        // - this should take an instance file and turn it into the set of constants
    }

    //parseConstants should be called before this is called
    private void addVariables() throws IloException {


        x = new IloIntVar[this.numCars][this.numPkgs][this.numPkgs];
        y = new IloIntVar[this.numCars][this.numDepots][this.numPkgs];
        z = new IloIntVar[this.numPkgs][this.numCars];
        beta = new IloIntVar[this.numCars][this.numDepots];

        t = new IloIntVar[this.numCars][this.numPkgs];




        //x vars
        for(int h = 0; h< this.numCars;h++){
            for(int i = 0; i < this.numPkgs; i++) {
                x[h][i] = cplex.intVarArray(this.numPkgs, 0, 1);
            }
        }

        //y vars
        for(int h = 0; h< this.numCars;h++){
            for(int i = 0; i< this.numDepots;i++) {
                y[h][i] = cplex.intVarArray(this.numPkgs, 0, 1);
            }
        }

        //z vars
        for(int i = 0; i<this.numPkgs;i++){
            z[i] = cplex.intVarArray(this.numCars,0,1);
        }

        //beta vars
        for(int i = 0; i<this.numCars;i++){
            beta[i] = cplex.intVarArray(this.numDepots,0,1);
        }


        //t vars
        for(int h = 0; h<this.numCars;h++){
            t[h] = cplex.intVarArray(this.numPkgs,0,1);
        }
    }


    private void addObjective() throws IloException {

        IloNumExpr obj = cplex.linearNumExpr();

        for(int h = 0; h< x.length; h++) {
            for(int i = 0; i < x[h].length; i++){
                obj = cplex.sum(obj,cplex.scalProd(c[i],x[h][i]));
            }
        }

        for(int h = 0; h< y.length; h++) {
            for(int i = 0; i < y[h].length; i++) {
                obj = cplex.sum(obj, cplex.scalProd(c1[i], y[h][i]));
            }
        }

        for(int i = 0; i< z.length; i++) {
            obj = cplex.sum(obj,cplex.scalProd(c2[i],z[i]));
        }

        for(int i = 0; i< beta.length; i++) {
            obj = cplex.sum(obj,cplex.scalProd(c3[i],beta[i]));
        }

        cplex.addMinimize(obj);

    }


    //each package must be delivered exactly once
    private void addConstraint1() throws IloException {

        for (int j = 0; j < this.numPkgs; j ++){
            //-------exactly one car enters pkg loc j--------

            //create lhs expression
            IloIntExpr entersj_lhs = cplex.linearIntExpr();
            for(int h = 0; h< this.numCars; h++) {
                for(int i = 0; i < this.numPkgs; i++) {
                    if (i != j) { //should ensure no loops back to itself
                        entersj_lhs = cplex.sum(entersj_lhs, x[h][i][j]);
                    }
                }

                for(int k = 0; k < this.numDepots;k++){
                    entersj_lhs = cplex.sum(entersj_lhs,y[h][k][j]);
                }
            }

            //add constraint to model
            cplex.addEq(entersj_lhs,1,"exactly one car enters pkg loc" + j);



            //-------exactly one car leaves pkg loc j-------

            //create lhs expression
            IloIntExpr exitsj_lhs = cplex.linearIntExpr();

            for(int h = 0; h< this.numCars; h++) {
                for(int i = 0; i < this.numPkgs; i++) {
                    if (i != j) {// should ensure no loops
                        exitsj_lhs = cplex.sum(exitsj_lhs, x[h][j][i]);
                    }
                }

                exitsj_lhs = cplex.sum(exitsj_lhs,z[j][h]);
            }
            cplex.addEq(exitsj_lhs,1,"exactly one car exits pkg loc" + j);

        }


    }

    //if a car h enters pkg loc j then it must also leave that location
    private void addConstraint2() throws IloException {
        for (int h = 0; h< this.numCars; h++){
            for (int j = 0; j < this.numPkgs;j++){

                //create lhs expression
                IloIntExpr lhs = cplex.linearIntExpr();

                for (int i = 0; i < this.numPkgs; i++){
                    lhs = cplex.sum(lhs,x[h][i][j]);
                }

                for (int k = 0; k < this.numDepots;k++){
                    lhs = cplex.sum(lhs,y[h][k][j]);
                }

                //create rhs expression
                IloIntExpr rhs = cplex.linearIntExpr();
                for (int l = 0; l < this.numPkgs; l++){
                    rhs = cplex.sum(rhs,x[h][j][l]);
                }
                rhs = cplex.sum(rhs,z[j][h]);


                //add constraint to model
                cplex.addEq(lhs,rhs,"if car " + h + "enters pkg loc " + j + "then it also leaves that location");

            }
        }

    }

    //each car goes first to at most one depot
    private void addConstraint3() throws IloException {

        for(int h = 0; h < this.numCars;h++) {
            //create lhs expression
            IloIntExpr lhs = cplex.linearIntExpr();
            for (int k = 0; k < this.numDepots; k++) {
                lhs = cplex.sum(lhs, beta[h][k]);
            }

            //add constraint to model
            cplex.addLe(lhs,1);
        }

    }


    //if a car enters a depot then it also leaves that depot
    private void addConstraint4() throws IloException {
        for(int h = 0; h < this.numCars;h++){
            for (int k = 0; k < this.numDepots;k++) {
                //create lhs expression
                IloIntExpr lhs = cplex.linearIntExpr();
                for (int i = 0; i < this.numPkgs;i++){
                    lhs = cplex.sum(lhs,y[h][k][i]);
                }


                //create rhs expression
                cplex.addEq(lhs,beta[h][k],"if car " + h + " enters depot " + k + " then it also has to leave that depot");

            }


        }
    }

    // a car visits at least one depot when it enters a depot
    // ensure SUM beta[h][k] = 1 when car h is used
    private void addConstraint5() throws IloException {
        //a car is used when it visits at least one package
        for(int h = 0; h<this.numCars;h++) {
            for(int i = 0; i<this.numPkgs;i++){

                //create lhs
                IloIntExpr lhs = cplex.linearIntExpr();
                for(int j = 0; j < this.numPkgs;j++){
                    lhs = cplex.sum(lhs,x[h][i][j]);
                }
                lhs = cplex.sum(lhs,z[i][h]);


                //create rhs
                IloIntExpr rhs = cplex.linearIntExpr();
                for(int k = 0; k<this.numDepots;k++){
                    rhs = cplex.sum(rhs,beta[h][k]);
                }


                //add expression to model

                cplex.addLe(lhs,rhs);
            }

        }


    }



    //if car h is used than it ends at its dest (\exists i s.t z_hi = 1)
    //TODO this constraint is redundent and we can eliminate it
    private void addConstraint7() throws IloException {
        for(int h = 0; h<this.numCars;h++){

            //lhs
            IloIntExpr lhs = cplex.linearIntExpr();
            for(int k= 0; k<this.numDepots;k++){
                lhs = cplex.sum(lhs,beta[h][k]);
            }


            //rhs
            IloIntExpr rhs = cplex.linearIntExpr();
            for(int i= 0; i<this.numPkgs;i++){
                rhs = cplex.sum(rhs,z[i][h]);
            }

            //add to model
            cplex.addEq(lhs,rhs,"if car " + h+" goes to a depot, then it also arrives at its desination");
        }
    }

    //MTZ cycle elimination constraint
    private void addConstraint8() throws IloException {
        for(int h = 0; h<this.numCars;h++){
            for (int i = 0; i < this.numPkgs; i++) {
                for (int j = 0; j < this.numPkgs; j++) {

                    //create rhs expression
                    int B = 2*this.numPkgs;// should be sufficiently large
                    IloIntExpr rhs;
                    rhs = cplex.sum(1,cplex.prod(-1,x[h][i][j]));
                    rhs = cplex.prod(-B,rhs);
                    rhs = cplex.sum(t[h][i],rhs);
                    rhs = cplex.sum(1,rhs);


                    //add to model
                    cplex.addGe(t[h][j],rhs, "MTZ cycle constraint");
                }

            }

        }
    }


    //each car can deliver at most n[h] packages
    private void addConstraint9(){
        //TODO
    }



    public void solve() throws IloException {
        cplex.solve();
    }

    public void printSolution() throws IloException {
        System.out.println(cplex.getObjValue());


        for (int h = 0; h < this.numCars; h++) {
            for (int i = 0; i < this.numPkgs; i++) {
                for (int j = 0; j < this.numPkgs; j++) {
                    String var_name = "x^"+ h + "_" + i + "_" + j;
                    System.out.println( var_name + ": " + cplex.getValue(x[h][i][j]));
                }
                System.out.println();
            }
        }

        for (int h = 0; h < this.numCars; h++) {
            for (int k = 0; k < this.numDepots; k++) {
                System.out.println("beta[" + h + "][" + k + "]: " + cplex.getValue(beta[h][k]));

            }
        }


        for (int h = 0; h < this.numCars; h++) {
            for (int i = 0; i < this.numPkgs; i++) {
                System.out.println("z[" + i + "][" + h + "]: " + cplex.getValue(z[i][h]));

            }
        }

        for(int h = 0; h<this.numCars; h++){
            for(int k = 0; k < this.numDepots; k++){
                for(int i = 0; i < this.numPkgs;i++){
                    String var_name = "y["+ h + "][" + k + "][" + i + "]";
                    System.out.println( var_name + ": " + cplex.getValue(y[h][k][i]));
                }
                System.out.println();
            }
        }


        for (int h = 0; h < this.numCars; h++) {
            for (int i = 0; i < this.numPkgs; i++) {
                System.out.println("t[" + h + "][" + i +"]: " + cplex.getValue(t[h][i]));

            }
        }

    }
}
