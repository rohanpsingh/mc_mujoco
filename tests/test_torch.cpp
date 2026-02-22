#include <torch/torch.h>
#include <cstdlib>
#include <iostream>

int main()
{
  // Basic tensor creation and arithmetic
  auto a = torch::tensor({1.0f, 2.0f, 3.0f});
  auto b = torch::tensor({4.0f, 5.0f, 6.0f});
  auto c = a + b;

  auto expected = torch::tensor({5.0f, 7.0f, 9.0f});
  if(!torch::allclose(c, expected))
  {
    std::cerr << "Tensor addition failed: " << c << std::endl;
    return EXIT_FAILURE;
  }

  // Matrix multiply
  auto mat = torch::ones({3, 3}) * 2.0f;
  auto vec = torch::ones({3, 1});
  auto result = torch::mm(mat, vec);
  auto expected_mm = torch::full({3, 1}, 6.0f);
  if(!torch::allclose(result, expected_mm))
  {
    std::cerr << "Matrix multiply failed: " << result << std::endl;
    return EXIT_FAILURE;
  }

  // Simple MLP forward pass (no training, just runtime inference)
  torch::nn::Sequential mlp(torch::nn::Linear(4, 8), torch::nn::ReLU(), torch::nn::Linear(8, 2));
  mlp->eval();
  auto input = torch::randn({1, 4});
  auto output = mlp->forward(input);
  if(output.sizes() != std::vector<int64_t>{1, 2})
  {
    std::cerr << "MLP output shape wrong: " << output.sizes() << std::endl;
    return EXIT_FAILURE;
  }

  std::cout << "All torch runtime tests passed." << std::endl;
  return EXIT_SUCCESS;
}
