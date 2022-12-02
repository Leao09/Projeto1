
//cria uma variável api com o endereço fornecido pelo node
api = 'http://127.0.0.1:3071'

//Faz com que a lista seja coletada antes de mostrar a página
$(document).ready(() => {
    users.list();
});

// cria uma variável users para receber a lista
var users = {
    //Chama a função list() para criar uma lista 
    //com os resultados vindos do servidor e chamados pelo ajax
    list() {
        $.ajax({
            //Valor da variável api somada com /users fica http://127.0.0.1:3071/users
            url: api + '/users',
            //Tipo de requisição HTTP - GET de pegar/buscar dados
            type: 'GET',
            //quando bem sucedido a resposta do servidor é jogado dentro de data
            //data terá várias linhas
            success: data => {
                //tx é uma variável de texto que será usada para concatenar campos
                //do html junto com o campo TITLE proveniente da resposta do servidor
                var tx = '';
                //Cria-se um looping para pegar todas as linhas da variável data
                //a variável element captura uma linha por vez deêm um console.log(element)
                //para ver o que ela imprime
                data.forEach(element => {
                    tx += '<div class="user">';
                    //no meio do texto fixo, adiciona-se o campo title da linha capturada
                    //pela variável element
                    tx += '<div class="title">' + element.title + '</div>';
                    tx += '<div class="actions">';
                    tx += '</div>';
                    tx += '</div>';
                });
                //através do jquery carregamos/substituímos o objeto de ID
                //main pelos códigos concatenados acima.
                $('#main').html(tx);
            }
        });
        
    }
    
    
};

//criamos uma função para rodar através do botão
function teste(){
    //Similar ao AJAX essa é a função GET do JQUERY
    //o get irá acessar o servidor no link http://127.0.0.1:3071/users
    //o resultado será guardado na variável resultado
    $.get("http://127.0.0.1:3071/users", function(resultado){
        //repete o mesmo processo do AJAX
        var tx = ' '
        resultado.forEach(element => {
            tx += '<div class="user">';
                        tx += '<div class="title">' + element.title + '</div>';
                        tx += '<div class="actions">';
                        tx += '</div>';
                    tx += '</div>';
        });
        //Diferentemente do AJAX este troca o o conteúdo do ID main2
        $('#main2').html(tx);
    });
}
