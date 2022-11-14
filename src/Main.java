
import ilog.concert.*;
import ilog.cplex.*;



import java.util.zip.DeflaterOutputStream;

public class Main {


    public static void main(String[] args) throws IloException {



        // mini example to get cplex working
        /*
        min 0.12x_1+0.15x_2
        subject to:
        60x_1 + 60x_2 >= 300
        12x_1 +6 x_2 >= 36
        10x_1 + 30x_2 >= 90
        x_1 >=0
        x_2 >= 0
         */


        //init model
        IloCplex cplex = new IloCplex();

        //variables
        IloNumVar x_1 = cplex.numVar(0, Double.MAX_VALUE, "x_1");
        IloNumVar x_2 = cplex.numVar(0, Double.MAX_VALUE, "x_2");

        //objective:
        IloLinearNumExpr obj = cplex.linearNumExpr();
        obj.addTerm(0.12, x_1);
        obj.addTerm(0.15,x_2);

        cplex.addMinimize(obj);

        //constraints

        cplex.addGe(cplex.sum(cplex.prod(60,x_1),cplex.prod(60,x_2)),300); // ad greater than or equal cosntraint
        cplex.addGe(cplex.sum(cplex.prod(12,x_1),cplex.prod(6,x_2)),36);
        cplex.addGe(cplex.sum(cplex.prod(10,x_1),cplex.prod(30,x_2)),90);

        //solve
        cplex.solve();

        System.out.println(cplex.getObjValue());
        System.out.println(cplex.getValue(x_1));
        System.out.println(cplex.getValue(x_2));


    }
}
