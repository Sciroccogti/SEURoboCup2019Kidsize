#include "gamectrl.hpp"
#include <map>
#include <fstream>
#include <simulation/Stand.h>

using namespace std;

map<int, string> states = {
    {simulation::GameData::STATE_INIT, "Init"},
    {simulation::GameData::STATE_READY, "Ready"},
    {simulation::GameData::STATE_PLAY, "Play"},
    {simulation::GameData::STATE_PAUSE, "Pause"},
    {simulation::GameData::STATE_END, "End"},
};


TeamLabel::TeamLabel(QString name, TeamColor color)
{
    colorLabel = new QLabel();
    colorLabel->setFixedWidth(16);
    QString cstr = color==TEAM_RED?"background-color:red;":"background-color:blue";
    colorLabel->setStyleSheet(cstr);
    nameLabel = new QLabel(name);
    nameLabel->setStyleSheet("font-size:20px;");
    scoreLabel = new QLabel("0");
    scoreLabel->setFixedWidth(30);
    scoreLabel->setAlignment(Qt::AlignCenter);
    scoreLabel->setStyleSheet("font-size:20px;");

    QHBoxLayout *lay = new QHBoxLayout();
    lay->addWidget(colorLabel);
    lay->addWidget(nameLabel);
    lay->addWidget(scoreLabel);
    this->setLayout(lay);
}

StartDlg::StartDlg(std::string cfg)
{
    string line;
    QStringList teams;
    ifstream ifs(cfg);
    if(ifs){
        while(!ifs.eof()){
            ifs>>line;
            if(!line.empty() && line.back()=='\n') 
                line.pop_back();
            if(!line.empty())
                teams<<QString::fromStdString(line);
        }
        ifs.close();
    }
    QVBoxLayout *leftLayout, *rightLayout;
    leftLayout = new QVBoxLayout;
    rightLayout = new QVBoxLayout;
    QLabel *redLabel = new QLabel();
    redLabel->setStyleSheet("background-color: red;");
    QLabel *blueLabel = new QLabel();
    blueLabel->setStyleSheet("background-color: blue;");
    redTeamBox = new QComboBox();
    blueTeamBox = new QComboBox();
    redTeamBox->addItems(teams);
    blueTeamBox->addItems(teams);
    leftLayout->addWidget(redLabel);
    leftLayout->addWidget(redTeamBox);
    rightLayout->addWidget(blueLabel);
    rightLayout->addWidget(blueTeamBox);

    startBtn = new QPushButton("Start");
    connect(startBtn, &QPushButton::clicked, this, &StartDlg::OnStart);

    QVBoxLayout *mainLayout = new QVBoxLayout;
    QHBoxLayout *upLayout = new QHBoxLayout;
    upLayout->addLayout(leftLayout);
    upLayout->addLayout(rightLayout);
    mainLayout->addLayout(upLayout);
    mainLayout->addWidget(startBtn);
    setLayout(mainLayout);
}

void StartDlg::OnStart()
{
    redName = redTeamBox->currentText();
    blueName = blueTeamBox->currentText();
    if(redName.size()==0 || blueName.size()==0)
        return;
    if(redName == blueName){
        QMessageBox::warning(this, "Error", "Two teams is the same!");
        return;
    }
    this->close();
}

GameCtrl::GameCtrl(QString red, QString blue)
{
    redName = red;
    blueName = blue;
    QHBoxLayout *mainLayout = new QHBoxLayout();
    QVBoxLayout *leftLayout, *rightLayout;
    leftLayout = new QVBoxLayout();
    rightLayout = new QVBoxLayout();

    timeLabel = new QLabel(QString::number(totalTime));
    timeLabel->setAlignment(Qt::AlignCenter);
    timeLabel->setStyleSheet("font-size:36px;");
    stimeLabel = new QLabel();
    stimeLabel->setAlignment(Qt::AlignCenter);
    stimeLabel->setStyleSheet("font-size:24px;");
    stateLabel = new QLabel("Init");
    stateLabel->setAlignment(Qt::AlignCenter);
    stateLabel->setStyleSheet("font-size:36px;");
    redTeam = new TeamLabel(red, TEAM_RED);
    blueTeam = new TeamLabel(blue, TEAM_BLUE);
    btnRedStand = new QPushButton("Red Stand Up");
    btnRedStand->setStyleSheet("font-size:20px;");
    btnBlueStand = new QPushButton("Blue Stand Up");
    btnBlueStand->setStyleSheet("font-size:20px;");

    leftLayout->addWidget(timeLabel);
    leftLayout->addWidget(stimeLabel);
    leftLayout->addWidget(stateLabel);
    leftLayout->addWidget(redTeam);
    leftLayout->addWidget(btnRedStand);
    leftLayout->addWidget(blueTeam);
    leftLayout->addWidget(btnBlueStand);

    stateInitBtn = new QPushButton("Init");
    stateInitBtn->setMinimumHeight(40);
    stateInitBtn->setStyleSheet("font-size:20px;");
    rightLayout->addWidget(stateInitBtn);
    stateReadyBtn = new QPushButton("Ready");
    stateReadyBtn->setMinimumHeight(40);
    stateReadyBtn->setStyleSheet("font-size:20px;");
    rightLayout->addWidget(stateReadyBtn);
    statePlayBtn = new QPushButton("Play");
    statePlayBtn->setMinimumHeight(40);
    statePlayBtn->setStyleSheet("font-size:20px;");
    rightLayout->addWidget(statePlayBtn);
    statePauseBtn = new QPushButton("Pause");
    statePauseBtn->setMinimumHeight(40);
    statePauseBtn->setStyleSheet("font-size:20px;");
    rightLayout->addWidget(statePauseBtn);
    stateFinishBtn = new QPushButton("Finish");
    stateFinishBtn->setMinimumHeight(40);
    stateFinishBtn->setStyleSheet("font-size:20px;");
    rightLayout->addWidget(stateFinishBtn);

    mainLayout->addLayout(leftLayout);
    mainLayout->addLayout(rightLayout);
    QWidget *mainWidget = new QWidget();
    mainWidget->setLayout(mainLayout);
    setCentralWidget(mainWidget);

    fTimer = new QTimer();
    sTimer = new QTimer();
    connect(fTimer, &QTimer::timeout, this, &GameCtrl::OnFTimer);
    connect(sTimer, &QTimer::timeout, this, &GameCtrl::OnSTimer);
    connect(stateInitBtn, &QPushButton::clicked, this, &GameCtrl::OnBtnInitClicked);
    connect(stateReadyBtn, &QPushButton::clicked, this, &GameCtrl::OnBtnReadyClicked);
    connect(statePlayBtn, &QPushButton::clicked, this, &GameCtrl::OnBtnPlayClicked);
    connect(statePauseBtn, &QPushButton::clicked, this, &GameCtrl::OnBtnPauseClicked);
    connect(stateFinishBtn, &QPushButton::clicked, this, &GameCtrl::OnBtnFinishClicked);
    connect(btnRedStand, &QPushButton::clicked, this, &GameCtrl::OnBtnRedStandClicked);
    connect(btnBlueStand, &QPushButton::clicked, this, &GameCtrl::OnBtnBlueStandClicked);

    fTimer->start(mBasicTIme);
    remainTime = totalTime;
    paused = true;
    mMs=0;
    readyRemainTime = readyTime;
    mGdata.state = simulation::GameData::STATE_INIT;
    mGdata.remainTime = totalTime;
    mNode = std::make_shared<ros::NodeHandle>();
    mGdataPublisher = mNode->advertise<simulation::GameData>("/gamedata", 1);
    mStandPublisher = mNode->advertise<simulation::Stand>("/stand", 1);
    mScoreSubscriber = mNode->subscribe("/judge/score", 1, &GameCtrl::ScoreUpdate, this);
    mTinfoServer = mNode->advertiseService("/teaminfo", &GameCtrl::TeamInfoService, this);
}

void GameCtrl::OnFTimer()
{
    ros::spinOnce();
    blueTeam->SetScore(mScore.blue);
    redTeam->SetScore(mScore.red);
    mGdata.blueScore = mScore.blue;
    mGdata.redScore = mScore.red;
    mGdata.remainTime = remainTime;
    mGdataPublisher.publish(mGdata);
    
    if(paused) return;
    mMs += mBasicTIme;
    if(mMs%1000==0){
        remainTime--;
        timeLabel->setText(QString::number(remainTime));
        if(remainTime<=0)
            OnBtnFinishClicked();
    }
}

void GameCtrl::OnSTimer()
{
    readyRemainTime--;
    stimeLabel->setText(QString::number(readyRemainTime));
    if(readyRemainTime == 0){
        sTimer->stop();
        OnBtnPlayClicked();
    }
}

void GameCtrl::OnBtnInitClicked()
{
    paused = true;
    mMs = 0;
    remainTime = totalTime;
    mGdata.state = simulation::GameData::STATE_INIT;
    stateLabel->setText(QString::fromStdString(states[mGdata.state]));
    timeLabel->setText(QString::number(remainTime));
}

void GameCtrl::OnBtnReadyClicked()
{
    sTimer->start(1000);
    readyRemainTime = readyTime;
    mGdata.state = simulation::GameData::STATE_READY;
    stateLabel->setText(QString::fromStdString(states[mGdata.state]));
    stimeLabel->setText(QString::number(readyRemainTime));
}

void GameCtrl::OnBtnPlayClicked()
{
    paused = false;
    mGdata.state = simulation::GameData::STATE_PLAY;
    stateLabel->setText(QString::fromStdString(states[mGdata.state]));
}

void GameCtrl::OnBtnPauseClicked()
{
    paused = true;
    mGdata.state = simulation::GameData::STATE_PAUSE;
    stateLabel->setText(QString::fromStdString(states[mGdata.state]));
}

void GameCtrl::OnBtnFinishClicked()
{
    paused = true;
    mGdata.state = simulation::GameData::STATE_END;
    stateLabel->setText(QString::fromStdString(states[mGdata.state]));
}

void GameCtrl::OnBtnRedStandClicked()
{
    simulation::Stand s;
    s.red=true;
    s.blue=false;
    mStandPublisher.publish(s);
}

void GameCtrl::OnBtnBlueStandClicked()
{
    simulation::Stand s;
    s.red=false;
    s.blue=true;
    mStandPublisher.publish(s);
}


void GameCtrl::ScoreUpdate(const game_ctrl::Score::ConstPtr &p)
{
    mScore = *(p.get());
    OnBtnReadyClicked();
}

bool GameCtrl::TeamInfoService(game_ctrl::TeamInfo::Request &req,
        game_ctrl::TeamInfo::Response &res)
{
    string tname = req.teamname;
    if(tname == redName.toStdString())
        res.team = "redrobot";
    else if(tname == blueName.toStdString())
        res.team = "bluerobot";
}