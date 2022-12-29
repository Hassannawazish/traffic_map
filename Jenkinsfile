pipeline {
    agent any
    stages {
        stage('Build') {
            steps {
                sh 'echo "Building......"'
                sh 'chmod +x script/ros_create.sh'
                sh 'script/ros_create.sh' 
                archiveArtifacts artifacts: 'bin/Debug/*', fingerprint: true
            }
        }
    }
}