import ilog.concert.IloException;

public class Test {


    public static void main(String[] args) throws IloException {

        MyModel model = new MyModel();
        model.solve();
        model.printSolution();


    }
}
