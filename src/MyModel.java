import ilog.concert.*;
import ilog.cplex.*;

import java.util.Random;

public class MyModel{
    private IloCplex cplex = new IloCplex();

    //variables
    private IloIntVar [][] x; //x[i][j] edge pkg i-> pkg j is used by some car
    private IloIntVar [][] y; //y[i][j] some vehicle leaves depot i to pkg j
    private IloIntVar [][] z; //z[i][j] car i serves package i last - i.e edge from pkg i to dest j is used
    private IloIntVar [][] alpha; //alpha[i][j] car i serves package j
    private IloIntVar [][] beta; //beta[i][j]  car i uses depot j - i.e edge from car i start to depot j is used

    //TODO the upper and lower bounds of these need to be defined - ie insure they are binary variables
    // -- todo maybe IloBoolVar works or something like that

    //constants
    private double [][] c; //c[i][j] cost (distance) of edge from pkg i to pkg j
    private double [][] c1;//c1[i][j] cost of edge from depot i to pkg j
    private double [][] c2; //c2[i][j] cost of edge from pkg i to car destination j
    private double [][] c3;//c3[i][j] cost of edge from car i start to depot j

    private int numPkgs;
    private int numCars;
    private int numDepots;

    public MyModel() throws IloException {
    //TODO
        //should init my model as written in my notes with all variables and constraints and the objective

        this.parseConstants();

        //***********************************//
        //using little test instance
        Random rand = new Random(2000);

        this.numPkgs = 4;
        this.numCars = 2;
        this.numDepots = 2;

        this.c = new double[this.numPkgs][this.numPkgs];
        this.c1 = new double[this.numDepots][this.numPkgs];
        this.c2 = new double[this.numPkgs][this.numCars];
        this.c3 = new double[this.numCars][this.numDepots];


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

        //*******************************//




        //populate

        this.addVariables();
        this.addObjective();
        this.addConstraint1();
        this.addConstraint2();
        this.addConstraint3();


    }

    private void addObjective() throws IloException {

        IloNumExpr obj = cplex.linearNumExpr();;

        for(int i = 0; i< x.length; i++) {
            obj = cplex.sum(obj,cplex.scalProd(c[i],x[i]));
        }

        for(int i = 0; i< y.length; i++) {
            obj = cplex.sum(obj,cplex.scalProd(c1[i],y[i]));
        }

        for(int i = 0; i< z.length; i++) {
            obj = cplex.sum(obj,cplex.scalProd(c2[i],z[i]));
        }

        for(int i = 0; i< beta.length; i++) {
            obj = cplex.sum(obj,cplex.scalProd(c3[i],beta[i]));
        }

        cplex.addMinimize(obj);

    }

    private void parseConstants(){
        //TODO
        //should take a instance file and turn it into the set of constants
    }


    //parseConstants should be called before this is called
    private void addVariables() throws IloException {

        //variables
        //private IloIntVar [][] x; //x[i][j] edge pkg i-> pkg j is used by some car
        //private IloIntVar [][] y; //y[i][j] some vehicle leaves depot i to pkg j
        //private IloIntVar [][] z; //z[i][j] car i serves package i last - i.e edge from pkg i to dest j is used
        //private IloIntVar [][] alpha; //alpha[i][j] car i serves package j
        //private IloIntVar [][] beta; //beta[i][j]  car i uses depot j - i.e edge from car i start to depot j is used

        x = new IloIntVar[this.numPkgs][this.numPkgs];
        y = new IloIntVar[this.numDepots][this.numPkgs];
        z = new IloIntVar[this.numPkgs][this.numCars];
        alpha = new IloIntVar[this.numCars][this.numPkgs];
        beta = new IloIntVar[this.numCars][this.numDepots];




        //x vars
        for(int i = 0; i< this.numPkgs;i++){
            x[i] = cplex.intVarArray(this.numPkgs,0,1);
        }

        //y vars
        for(int i = 0; i< this.numDepots;i++){
            y[i] = cplex.intVarArray(this.numPkgs,0,1);
        }

        //z vars
        for(int i = 0; i<this.numPkgs;i++){
            z[i] = cplex.intVarArray(this.numCars,0,1);
        }

        //alpha vars - might not be needed? \\TODO
        for(int i = 0; i<this.numCars;i++){
            alpha[i] = cplex.intVarArray(this.numPkgs,0,1);
        }

        for(int i = 0; i<this.numCars;i++){
            beta[i] = cplex.intVarArray(this.numDepots,0,1);
        }

    }

    //each package must be delivered exactly once
    private void addConstraint1() throws IloException {


        //TODO;
    }

    //each car go's first to exactly one depot and leaves the same depot once
    private void addConstraint2(){
        //TODO
    }

    //each car can deliver at most n packages
    private void addConstraint3(){
        //TODO
    }



    public void solve() throws IloException {
        cplex.solve();
    }

    public void printSolution() throws IloException {
        System.out.println(cplex.getObjValue());


        for(int i = 0; i < this.numPkgs;i++){
            for(int j = 0; j < this.numPkgs;j++){
                System.out.print("," + cplex.getValue(x[i][j]));
            }
            System.out.println();
        }
    }
}
