import java.util.Random;

import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

public class TesteQLearning {
	private static EV3LargeRegulatedMotor motorEsq = new EV3LargeRegulatedMotor(
			MotorPort.A);
	private static EV3LargeRegulatedMotor motorDir = new EV3LargeRegulatedMotor(
			MotorPort.D);
	private static EV3ColorSensor sensorEsq = new EV3ColorSensor(SensorPort.S1);
	private static EV3ColorSensor sensorDir = new EV3ColorSensor(SensorPort.S2);
	private static float corEsq;
	private static float corDir;

	private static final int Estado1 = 0; // Nenhum lê preto
	private static final int Estado2 = 1; // Só o direito lê preto
	private static final int Estado3 = 2; // Só o esquerdo lê preto
	private static final int numDeEstados = 3;
	private static int[] Estado = { Estado1, Estado2, Estado3 };

	private static final int Acao1 = 0; // Dois motores para frente
	private static final int Acao2 = 1; // Virar direita
	private static final int Acao3 = 2; // Virar esquerda
	private static final int numDeAcoes = 3;
	private static int[] Acao = { Acao1, Acao2, Acao3 };

	private static int[][][] Recompensa = new int[numDeEstados][numDeAcoes][numDeEstados]; // R(s,a,s')

	private static double[][] Q = new double[numDeEstados][numDeAcoes]; // Q(s,a)

	final static double alfa = 0.5; //Taxa de aprendizagem
	final static double gama = 0.5; //Fator de desconto

	public static void main(String[] args) {
		setVelocidade();
		while (!Button.ENTER.isDown()) {
			System.out.println("Aperte ENTER para inciar");
		}
		QLearning();
	}

	private static void inicializaRecompensas() {
		for (int i = 0; i < numDeEstados; i++) {
			for (int j = 0; j < numDeAcoes; j++) {
				for (int k = 0; k < numDeEstados; k++) {
					Recompensa[i][j][k] = -1;
				}
			}
		}
		/*
		 * A recompensa partindo de um estado, tomando uma acao e caindo em um
		 * novo estado
		 */
		Recompensa[Estado1][Acao1][Estado1] = 100;
		Recompensa[Estado2][Acao3][Estado1] = 99;
		Recompensa[Estado3][Acao2][Estado1] = 99;
		Recompensa[Estado2][Acao2][Estado1] = -100;
		Recompensa[Estado3][Acao3][Estado1] = -100;
	}

	private static void inicializaQ() {
		for (int i = 0; i < numDeEstados; i++) {
			for (int j = 0; j < numDeAcoes; j++) {
				Q[i][j] = 0;
			}
		}
	}

	private static int gerarAcao() {
		Random rand = new Random();
		int indice = rand.nextInt(numDeAcoes);
		return Acao[indice];
	}

	private static void agir(int acao) {
		if (acao == Acao1) {
			andarFrente();
		} else if (acao == Acao2) {
			virarEsq();
		} else if (acao == Acao3) {
			virarDir();
		}
	}
	
	private static void setVelocidade(){
		motorEsq.setSpeed(300);
		motorDir.setSpeed(300);
	}

	private static void andarFrente() {
		motorEsq.forward();
		motorDir.forward();
		Delay.msDelay(100);
	}

	private static void virarEsq() {
		motorEsq.backward();
		motorDir.forward();
		Delay.msDelay(100);
	}

	private static void virarDir() {
		motorDir.backward();
		motorEsq.forward();
		Delay.msDelay(100);
	}

	private static int verificarEstado() {
		amostraCor();
		if (corEsq == 7 && corDir != 7)
			return Estado3;
		else if (corEsq != 7 && corDir == 7)
			return Estado2;
		else
			return Estado1;
	}

	private static void QLearning() {
		inicializaRecompensas(); // inicializa os valores das recompensas
		inicializaQ(); // seta todos os elementos da matriz Q como zero
		int estado = verificarEstado();
		
		//Faz acoes aleatorias e preenche na tabela Q
		for (int i = 0; i < 50; i++) {
			int acao = gerarAcao();
			agir(acao);
			int proxEstado = verificarEstado();

			double q = Q[estado][acao];
			double maxQ = maxQ(proxEstado);
			int r = getR(estado, acao, proxEstado);

			double valor = q + alfa * (r + gama * maxQ - q);
			setQ(estado, acao, valor);
			System.out.println("Q(" + estado + "," + acao + ") = "
					+ Q[estado][acao]);
			estado = proxEstado;
		}
		Sound.beepSequenceUp();
		//Faz as melhores ações de acordo com o que preencheu na tabela Q
		while (!Button.ESCAPE.isDown()) {
			estado = verificarEstado();
			int acao = escolherAcao(estado);
			agir(acao);
			int proxEstado = verificarEstado();

			double q = Q[estado][acao];
			double maxQ = maxQ(proxEstado);
			int r = getR(estado, acao, proxEstado);

			double valor = q + alfa * (r + gama * maxQ - q);
			setQ(estado, acao, valor);
			System.out.println("Q(" + estado + "," + acao + ") = "
					+ Q[estado][acao]);
			estado = proxEstado;
		}
	}

	private static void setQ(int estado, int acao, double valor) {
		Q[estado][acao] = valor;
	}

	private static int getR(int estado, int acao, int proxEstado) {
		return Recompensa[estado][acao][proxEstado];
	}

	private static double maxQ(int proxEstado) {
		double maxQ = Double.MIN_VALUE;
		for (int i = 0; i < numDeAcoes; i++) {
			int proxAcao = Acao[i];
			double valor = Q[proxEstado][proxAcao];
			if (valor > maxQ)
				maxQ = valor;
		}
		return maxQ;
	}
	
	private static int escolherAcao(int estado){
		double maxQ = Double.MIN_VALUE;
		int melhorAcao = Integer.MIN_VALUE;
		for (int i = 0; i < numDeAcoes; i++) {
			int acao = Acao[i];
			double valor = Q[estado][acao];
			if (valor > maxQ) {
				maxQ = valor;
				melhorAcao = acao;
			}
		}
		return melhorAcao;
	}

	private static void amostraCor() {
		SampleProvider coresEsq = sensorEsq.getColorIDMode();
		SampleProvider coresDir = sensorDir.getColorIDMode();
		float amostraCorEsq[] = new float[sensorEsq.sampleSize()];
		float amostraCorDir[] = new float[sensorDir.sampleSize()];
		coresEsq.fetchSample(amostraCorEsq, 0);
		coresDir.fetchSample(amostraCorDir, 0);
		corEsq = amostraCorEsq[0];
		corDir = amostraCorDir[0];
	}
}
