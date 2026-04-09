function Vissim = VissimInitconfig(filepath)

    fprintf('\n'); % (
    fprintf('======================================================\n');
    fprintf('      [VissimInitconfig] 시뮬레이션 로드 중... \n');
    fprintf('======================================================\n');

    % VissimInitconfig Initialize Vissim COM server and load network
    status = system('taskkill /F /IM Vissim.exe');
    if status == 0
        disp('>> 기존에 실행 중이던 Vissim 프로세스를 강제 종료했습니다.');
    else
        disp('>> 실행 중인 Vissim 프로세스가 없습니다 (새로 시작합니다).');
    end

    feature ('COM_SafeArraySingleDim', 1);

    Vissim = actxserver('Vissim.Vissim');
    disp('>> Vissim이 실행되었습니다.');

    [pathstr, ~, ~] = fileparts(filepath);
    if isempty(pathstr)
        FullFilePath = fullfile(pwd, filepath);     % pwd: Print Working Directory (현재 경로)
    else
        FullFilePath = filepath;
    end
    
    if ~isfile(FullFilePath)
        error(['지정된 파일을 찾을 수 없습니다: ', FullFilePath]);
    end

    % 파일 로드
    disp(['>> 네트워크 로드: ', FullFilePath]);

    Vissim.LoadNet(FullFilePath, false);
    fprintf('>> [VissimInitconfig] 시뮬레이션 로드 완료.\n');
    fprintf('======================================================\n');

end